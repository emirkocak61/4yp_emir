cmake_minimum_required(VERSION 3.0.2)
project(anytree_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  exotica_core
  exotica_python
  exotica_core_task_maps
  exotica_ik_solver
  exotica_nlopt_solver
  sensor_msgs
  geometry_msgs
  rospy
  message_generation
  actionlib
  actionlib_msgs
  bt_drs_msgs
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
    bt_drs_msgs    
)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES motion_planning_exotica
CATKIN_DEPENDS sensor_msgs std_msgs actionlib_msgs geometry_msgs bt_drs_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
 ${catkin_INCLUDE_DIRS}
 )

add_executable(approach_target_as src/approachtarget_as.cpp src/MotionPlannerBaseClass.cpp)
add_dependencies(approach_target_as ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(approach_target_as
  ${catkin_LIBRARIES}
  # other libraries your executable depends on
)

function(CompileExample name)
  add_executable(${name} src/examples/${name}.cpp)
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} PUBLIC ${catkin_LIBRARIES})
endfunction()

CompileExample(example_motion)

## Add cmake target dependencies of the executable
## Specify libraries to link a library or executable target against
## Specify libraries to link a library or executable target against


