cmake_minimum_required(VERSION 2.8.3)
project(example_plugin)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For example_plugin plugin
add_library(hello_world src/hello_world.cc)
target_link_libraries(hello_world ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# # For model_push_plugin plugin
add_library(model_push_plugin src/model_push.cc)
target_link_libraries(model_push_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# # For earthquake_plugin plugin
# add_library(earthquake_plugin src/earthquake.cc)
# target_link_libraries(earthquake_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)