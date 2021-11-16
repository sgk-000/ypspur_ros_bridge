#include <cmath>                     // for math PI
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // for joint_state
#include <nav_msgs/msg/odometry.hpp> // for odometry
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
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

  // set twist publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;

  // set pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

  // set joint_state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
  // set tf broad caster
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  // ros timer
  rclcpp::Time current_time;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  
private:
};

YpspurROSBridgeOdomPublisher::YpspurROSBridgeOdomPublisher() :
  rclcpp::Node("ypspur_ros_bridge_odom_publisher"),
  tf2_broadcaster_(*this),
  x(0.0),
  y(0.0),
  th(0.0),
  vx(0.0),
  vy(0.0),
  vth(0.0)
{
  left_wheel_joint = this->declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
  right_wheel_joint = this->declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
  frame_id = this->declare_parameter<std::string>("frame_id", "odom");
  child_frame_id = this->declare_parameter<std::string>("child_frame_id", "base_link");

  // odom publisher
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS{10});

  // twist publisher
  twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});

  // pose publisher
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::QoS{10});

  // joint_state publisher
  js_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS{10});
  odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  current_time = this->now();

  if(Spur_init() < 0)
    RCLCPP_ERROR(this->get_logger(), "Can't open spur");
}
