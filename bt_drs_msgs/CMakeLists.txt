cmake_minimum_required(VERSION 3.0.2)
project(bt_drs_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
    interruptManipulation.srv
    selectStrategyIncrementing.srv
    selectStrategyForceTorque.srv
    recordManipulationData.srv
    getHandleAngle.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    approachTarget.action
    graspTarget.action
    manipulateTarget.action
    testBase.action
    toggleDevice.action
    undockBase.action

    PlanIK.action
    PlanTrajectory.action
    PlanAffordanceActuation.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
    trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
)
