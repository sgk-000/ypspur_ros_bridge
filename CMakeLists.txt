cmake_minimum_required(VERSION 3.5)
project(ypspur_ros_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
   /usr/local/include
)

link_directories(
  /usr/local/lib
)
## Declare a cpp executable
ament_auto_add_executable(ypspur_ros_bridge_driver_node src/ypspur_ros_bridge_driver_node.cpp)
target_link_libraries(ypspur_ros_bridge_driver_node ypspur)

ament_auto_add_executable(ypspur_ros_bridge_odom_publisher_node src/ypspur_ros_bridge_odom_publisher_node.cpp)
target_link_libraries(ypspur_ros_bridge_odom_publisher_node ypspur)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)