#include <geometry_msgs/msg/twist.hpp> // for cmd_vel
#include <cmath>                       // for math PI
#include <rclcpp/rclcpp.hpp>           // for ros
#include <ypspur.h>                    // for yp-spur

class YpspurROSBridgeDriver : public rclcpp::Node
{
public:
  YpspurROSBridgeDriver();
  ~YpspurROSBridgeDriver();

private:
  // Call back
  void Callback(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel);
  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  // linear & angular limits
  double linear_vel_max;
  double angular_vel_max;
  double linear_acc_max;
  double angular_acc_max;

};

YpspurROSBridgeDriver::YpspurROSBridgeDriver()
    : rclcpp::Node("ypspur_ros_bridge_driver_node"),
      linear_vel_max(1.1), angular_vel_max(M_PI), linear_acc_max(1.0),
      angular_acc_max(M_PI) {
  // init yp-spur---------------------------------------------------------------------
  if( Spur_init() < 0)
    RCLCPP_ERROR(this->get_logger(), "can't open ypspur");

  // using parameter server
  linear_vel_max = this->declare_parameter<double>("linear_vel_max", linear_vel_max);
  angular_vel_max = this->declare_parameter<double>("angular_vel_max", angular_vel_max);
  linear_acc_max = this->declare_parameter<double>("linear_acc_max", linear_acc_max);
  angular_acc_max = this->declare_parameter<double>("angular_acc_max", angular_acc_max);

  // init velocity & accelaration limits (Unit is m/s & rad/s)
  Spur_set_vel(linear_vel_max);
  Spur_set_accel(linear_acc_max);
  Spur_set_angvel(angular_vel_max);
  Spur_set_angaccel(angular_acc_max);
  // end init yp-spur------------------------------------------------------------------

  cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
      std::bind(&YpspurROSBridgeDriver::Callback, this, std::placeholders::_1));
}

YpspurROSBridgeDriver::~YpspurROSBridgeDriver()
{
  Spur_stop();
  Spur_free();
  RCLCPP_INFO(this->get_logger(), "Stop the robot");

}

void YpspurROSBridgeDriver::Callback(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel){
  Spur_vel(cmd_vel->linear.x, cmd_vel->angular.z);
}
