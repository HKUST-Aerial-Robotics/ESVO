cmake_minimum_required(VERSION 2.8.3)
project(events_repacking_helper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

# make the executable
cs_add_executable(EventMessageEditor src/EventMessageEditor.cpp)
target_link_libraries(EventMessageEditor ${catkin_LIBARIES})