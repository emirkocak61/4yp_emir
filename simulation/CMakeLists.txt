cmake_minimum_required(VERSION 3.0.2)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs actionlib_msgs message_runtime
)

# TODO: catkin_install_python scripts
# TODO: install resources, robots [or deprecate]

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  