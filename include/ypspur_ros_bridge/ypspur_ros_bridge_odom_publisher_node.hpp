#include <cmath>                     // for math PI
#include <nav_msgs/msg/odometry.hpp> // for odometry
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // for joint_state
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ypspur.h>                        // for yp-spur

class YpspurROSBridgeOdomPublisher : public rclcpp::Node
{
public:
  YpspurROSBridgeOdomPublisher();

  // odometry pose infomation
  double x;
  double y;
  double th;

  // velocity twist information
  double vx;
  double vy;
  double vth;

  // wheel angular informtaion
  double l_ang;
  double r_ang;

  // wheel joint names
  std::string left_wheel_joint;
  std::string right_wheel_joint;

  // odom frame names
  std::string frame_id;
  std::string child_frame_id;

  // set odometory publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  // set joint_state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
  // set tf broad caster
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  // ros timer
  rclcpp::Time current_time;

private:
};

YpspurROSBridgeOdomPublisher::YpspurROSBridgeOdomPublisher() :
  rclcpp::Node("ypspur_ros_bridge_odom_publisher_node"),
  x(0.0),
  y(0.0),
  th(0.0),
  vx(0.0),
  vy(0.0),
  vth(0.0),
  left_wheel_joint("left_wheel_joint"),
  right_wheel_joint("right_wheel_joint"),
  frame_id("odom"),
  child_frame_id("base_link")
{
  left_wheel_joint = this->declare_parameter<std::string>("left_wheel_joint", left_wheel_joint);
  right_wheel_joint = this->declare_parameter<std::string>("right_wheel_joint", right_wheel_joint);
  frame_id = this->declare_parameter<std::string>("frame_id", frame_id);
  child_frame_id = this->declare_parameter<std::string>("child_frame_id", child_frame_id);

  // odom publisher
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1).transient_local());
  // joint_state publisher
  js_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(1).transient_local());

  current_time = this->now();

  if(Spur_init() < 0)
    RCLCPP_ERROR(this->get_logger(), "Can't open spur");
}
