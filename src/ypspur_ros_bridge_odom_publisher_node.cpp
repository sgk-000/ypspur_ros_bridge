#include "ypspur_ros_bridge/ypspur_ros_bridge_odom_publisher_node.hpp"  // include local header

int main(int argc, char** argv){

  // init ros
  rclcpp::init(argc, argv);
  // original class
  YpspurROSBridgeOdomPublisher YRBOdomPub;

  // for quaternion
  geometry_msgs::msg::Quaternion odom_quat;
  // for transform tf
  geometry_msgs::msg::TransformStamped odom_trans;
  // for pusblish odometry
  nav_msgs::msg::Odometry odom;
  // for pusblish twist
  geometry_msgs::msg::TwistStamped twist;
  // for pusblish pose
  geometry_msgs::msg::PoseStamped pose;
  // for publish joint_state
  sensor_msgs::msg::JointState js;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  // initialize messages
  YRBOdomPub.current_time = ros_clock.now();

  odom_trans.header.stamp = YRBOdomPub.current_time;
  odom_trans.header.frame_id = YRBOdomPub.frame_id;
  odom_trans.child_frame_id = YRBOdomPub.child_frame_id;

  odom.header.stamp = YRBOdomPub.current_time;
  odom.header.frame_id = YRBOdomPub.frame_id;
  odom.child_frame_id = YRBOdomPub.child_frame_id;

  twist.header.stamp = YRBOdomPub.current_time;
  twist.header.frame_id = "base_link";

  pose.header.stamp =YRBOdomPub.current_time;
  pose.header.frame_id = "odom";

  js.name.push_back(YRBOdomPub.left_wheel_joint);
  js.name.push_back(YRBOdomPub.right_wheel_joint);
  js.position.resize(2);

  // loop rate is 25.0 [Hz]
  rclcpp::Rate r(50.0);

  ///////////////////////////////////////////////////
  // LOOP START
  ///////////////////////////////////////////////////
  while(rclcpp::ok()){
    // get current time
    YRBOdomPub.current_time = ros_clock.now();

    // Get odometory pose information
    Spur_get_pos_GL(&YRBOdomPub.x, &YRBOdomPub.y, &YRBOdomPub.th);

    // get current velocity
    Spur_get_vel(&YRBOdomPub.vx, &YRBOdomPub.vth);

    // get current angular
    YP_get_wheel_ang(&YRBOdomPub.l_ang, &YRBOdomPub.r_ang);

    // TF transform section -------------------------------------------------------
    //first, we'll publish the transform over tf
    tf2::Quaternion yp_quaternion;
    yp_quaternion.setRPY(0, 0, YRBOdomPub.th);
    odom_quat = tf2::toMsg(yp_quaternion);

    odom_trans.header.stamp = YRBOdomPub.current_time;
    odom_trans.transform.translation.x = YRBOdomPub.x;
    odom_trans.transform.translation.y = YRBOdomPub.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    YRBOdomPub.tf2_broadcaster_.sendTransform(odom_trans);
    // TF transform section end ---------------------------------------------------

    // Odom section ---------------------------------------------------------------
    //next, we'll publish the odometry message over ROS
    //set the position
    odom.pose.pose.position.x = YRBOdomPub.x;
    odom.pose.pose.position.y = YRBOdomPub.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = YRBOdomPub.vx;
    odom.twist.twist.linear.y = YRBOdomPub.vy;
    odom.twist.twist.angular.z = YRBOdomPub.vth;

    //publish the message
    YRBOdomPub.odom_pub->publish(odom);
    // Odom section end -----------------------------------------------------------

    twist.header.stamp = YRBOdomPub.current_time;
    twist.twist.linear.x = YRBOdomPub.vx;
    twist.twist.linear.y = YRBOdomPub.vy;
    twist.twist.angular.z = YRBOdomPub.vth;
    YRBOdomPub.twist_pub->publish(twist);

    pose.pose.position = odom.pose.pose.position;
    pose.pose.orientation = odom.pose.pose.orientation;
    YRBOdomPub.pose_pub->publish(pose);  

    // Joint State section --------------------------------------------------------
    // at the end, we'll get the
    js.header.stamp = YRBOdomPub.current_time;
    js.name.push_back(YRBOdomPub.left_wheel_joint);
    js.name.push_back(YRBOdomPub.right_wheel_joint);
    js.position.push_back(YRBOdomPub.l_ang);
    js.position.push_back(YRBOdomPub.r_ang);
    YRBOdomPub.js_pub->publish(js);
    // Joint State section end ----------------------------------------------------

    r.sleep();
  }
  /////////////////////////////////////////
  // END LOOP
  ////////////////////////////////////////
}

