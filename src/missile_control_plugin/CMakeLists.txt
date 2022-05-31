cmake_minimum_required(VERSION 2.8.3)
project(missile_control_plugin)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  exocet_msgs
  geometry_msgs
  std_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
# For missile_control plugin
add_library(missile_control src/missile_control_plugin.cc)
target_link_libraries(missile_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(missile_control exocet_msgs_generate_messages_cpp)

catkin_package(
  CATKIN_DEPENDS exocet_msgs geometry_msgs std_msgs
  DEPENDS 
    roscpp 
    gazebo_ros 
)