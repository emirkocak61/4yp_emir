cmake_minimum_required(VERSION 3.0.2)
project(anytree_control_interface)

## Compile as C++14
 add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   geometry_msgs#   sensor_msgs#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES z1_interface
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp sensor_msgs std_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   $ENV{HOME}/catkin_ws/src/unitree_ros/z1_sdk/include
   $ENV{HOME}/catkin_ws/src/unitree_ros/z1_controller/config
   $ENV{HOME}/catkin_ws/src/unitree_ros/z1_controller/include
   /usr/include/eigen3

)


add_executable(z1_control_interface src/z1_lowcmd_ros.cpp)
target_link_libraries(z1_control_interface 
  ${catkin_LIBRARIES}
  ~/catkin_ws/src/unitree_ros/z1_sdk/lib/libZ1_SDK_x86_64.so
)
