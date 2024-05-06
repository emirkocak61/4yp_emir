cmake_minimum_required(VERSION 3.0.2)
project(anytree_control)

## Compile as C++14
 add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  anytree_motion_planner
  exotica_core
  exotica_python
  exotica_core_task_maps
  actionlib
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  anytree_msgs
)

find_package(pinocchio REQUIRED)

# NOTE: Also relies on z1_controller and z1_sdk, but these can't be built like normal packages...
#       (Hence the strange unitree-related includes below)


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
  INCLUDE_DIRS include
#  LIBRARIES z1_interface
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp sensor_msgs std_msgs tf2_ros
  DEPENDS pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../z1_sdk/include
  ${PROJECT_SOURCE_DIR}/../../z1_controller/config
  ${PROJECT_SOURCE_DIR}/../../z1_controller/include
  /usr/include/eigen3
)
# TODO: improve include definitions

function(Compile name)
  add_executable(${name} src/${name}.cpp)
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} 
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../../z1_sdk/lib/libZ1_SDK_${CMAKE_HOST_SYSTEM_PROCESSOR}.so)
endfunction()

Compile(unitree)
Compile(UnitreeSim)
Compile(UnitreeHw)

