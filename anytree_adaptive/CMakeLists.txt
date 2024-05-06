cmake_minimum_required(VERSION 3.0.2)
project(anytree_adaptive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
  template_matcher
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs actionlib_msgs message_runtime
)

  