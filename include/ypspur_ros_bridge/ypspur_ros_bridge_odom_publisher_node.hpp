#include <ros/ros.h>                    // for ros
#include <tf/transform_broadcaster.h>	// for tf
#include <nav_msgs/Odometry.h>	        // for odometry
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>     // for joint_state
#include <ypspur.h>		                // for yp-spur
#include <cmath>	 	                // for math PI

class YpspurROSBridgeOdomPublisher
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
  
  // set node handler
  ros::NodeHandle n;
  
  // set odometory publisher
  ros::Publisher odom_pub;

  // set joint_state publisher
  ros::Publisher js_pub;
  
  // set twist publisher
  ros::Publisher twist_pub;

  // set twist publisher
  ros::Publisher twist_with_cov_pub;

  ros::Publisher pose_pub;

  ros::Publisher pose_with_cov_pub;

  // set tf broad caster
  tf::TransformBroadcaster odom_broadcaster;

  // ros timer
  ros::Time current_time;
  
private:
};

YpspurROSBridgeOdomPublisher::YpspurROSBridgeOdomPublisher() :
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
  ros::NodeHandle nh("~");
  
  nh.param("left_wheel_joint", left_wheel_joint, left_wheel_joint);
  nh.param("right_wheel_joint", right_wheel_joint, right_wheel_joint);
  nh.param("frame_id", frame_id, frame_id);
  nh.param("child_frame_id", child_frame_id, child_frame_id);
  
  
  
  // odom publisher
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

  // joint_state publisher
  js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  
  // joint_state publisher
  twist_pub = n.advertise<geometry_msgs::TwistStamped>("twist", 10);
  twist_with_cov_pub =
    n.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist_with_covariance", 10);
  
  pose_pub = n.advertise<geometry_msgs::TwistStamped>("pose", 10);
  pose_with_cov_pub =
    n.advertise<geometry_msgs::TwistWithCovarianceStamped>("pose_with_covariance", 10);
  
  current_time = ros::Time::now();
  
  if(Spur_init() < 0)
    ROS_ERROR("Can't open spur");
}
