#include "ypspur_ros_bridge/ypspur_ros_bridge_driver_node.hpp"

int main(int argc, char** argv){

  // init ros
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<YpspurROSBridgeDriver>());
}