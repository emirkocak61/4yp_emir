#!/usr/bin/env python3

import pyexotica as exo
from numpy import array, linalg
import math
import json
import rospkg
from pyexotica.publish_trajectory import publish_pose
from time import perf_counter
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import tf2_ros
from tf import transformations
from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_msgs.msg import Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rp = rospkg.RosPack()
package_path = rp.get_path("anytree_motion_planner")

__all__ = ["MPCMotionPlannerBaseClass"]


class MPCMotionPlannerBaseClass:
    #Base class for an MPC EXOTica-based motion planner, implemented as a ROS Action Server

    # Abstract base class, must implement:
    #   * Action-specific init, defining and starting action server
    #   * _feedback and _result action variables
    #   * callback for action server
    #   Also requires 'main' function OUTSIDE class definition, setting up the ROS node etc.
    def __init__(self):

        self.base_dof = 6 
        self.dt = 0.02 #Iteration time-step 0.02 corresponds to 50Hz control rate
        self.rate = rospy.Rate(1 / self.dt)

        self.start_tolerance = 5e-2 #Tolerance between th End Effector Frame(EEF) and Target at the start of the motion plan

        #For non-trajectory motion plans, the motion planner needs some indication of when to stop
        self.error_metric = "Position"
        self.tolerance = 2.5e-2 #Tolerance between EEF and Target
        self.counter = 10 #No. of consecutive iterations for which EEF is within tolerance
        self.t_limit = 10.0 #time limit - default 90

        #If the real arm can't follow the motion plan for maniplation, the motion plan should be stopped
        self.self_tolerance = 5e-2 #Tolerance between the motion plan and real EEF
        self.self_counter = 10 # No. consecutive iterations for which real EEF is outside tolerance

        # Initialize a ROS publisher for sending joint trajectory messages.
        #This publisher will send messages to the "/motion_plan" topic,
        # which can be used by other ROS nodes to receive motion plans for the robot.
        self.motion_plan_publisher = rospy.Publisher("/motion_plan",
                                                     JointTrajectory, 
                                                     queue_size=10,
                                                     tcp_nodelay=True
                                                     )
        # Initialize a TF listener to subscribe to transform updates and store
        # them in the tfBuffer.
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #Setup EXOTica
        self.solver = exo.Setup.load_solver(                #Returns a MotionSolver object
            "{anytree_motion_planner}/resources/configs/"
            + str(self.action_name)
            +".xml"
        )

        self.problem = self.solver.get_problem()    #Returns a PlanningProblem object
        self.scene = self.problem.get_scene()       #Returns a Scene object
        self.joint_names = self.scene.get_controlled_joint_names() #Returns a list[str]

        self.rest_pose = self.lookup_base_pose()

        self.kinematic_tree = self.scene.get_kinematic_tree() #Returns Kinematic Tree
        
        #Get the joint limits of the robot, which are minimum and maximum
        #values that each joint can reach
        joint_limits = self.kinematic_tree.get_joint_limits() #Returns a 2D array
        self.relative_joint_limits_upper = joint_limits[:,1]
        self.relative_joint_limits_lower = joint_limits[:,0]
        self.joint_limit_tolerance = 0.0001
        self.set_joint_limits()

        joint_velocities = self.kinematic_tree.get_velocity_limits()

        #Create a subscriber to allow external nodes to trigger a reset of robot's rest pose
        rospy.Subscriber("/reset_rest_pose",Bool,self.reset_rest_pose_callback)

        #Set cost for each forward-iteration in the finite horizon
        #In this case only terminal cost have a significant weight
        #Should hopefully allow planner to explore getting around local minima
        for t in range(self.problem.T):
            self.problem.cost.set_goal("Position", [0.0,0.0,0.0,0.0,0.0,0.0],t)
            self.problem.cost.set_rho("Position", 1e1,t)
        self.problem.cost.set_goal("Position", [0.0,0.0,0.0,0.0,0.0,0.0],-1)
        self.problem.cost.set_rho("Position", 1e1,-1)
        
        self.solver.debug_mode = False
        self.solver.max_iterations = 1
    
    def reset_rest_pose_callback(self,reset_rest_pose):
        if reset_rest_pose.data:
            self.rest_pose = self.lookup_base_pose() #Sets rest pose for anytree base
            self.set_joint_limits() #Sets joint limits according to new rest pose
    
    def set_joint_limits(self):
        self.scene.set_model_state_map(
            {
                "world_joint/trans_x" : self.rest_pose[0],
                "world_joint/trans_y" : self.rest_pose[1],
                "world_joint/trans_z" : self.rest_pose[2],
                "world_joint/rot_x" : self.rest_pose[3],
                "world_joint/rot_y" : self.rest_pose[4],
                "world_joint/rot_z" : self.rest_pose[5],
            }
        )
        global_joint_limits_upper = self.relative_joint_limits_upper.copy()
        global_joint_limits_lower = self.relative_joint_limits_lower.copy()

        for i in range(0,self.base_dof):
            global_joint_limits_upper[i] = (
                self.rest_pose[i] 
                + self.relative_joint_limits_upper[i] 
                + self.joint_limit_tolerance
            )
            global_joint_limits_lower[i] = (
                self.rest_pose[i] 
                + self.relative_joint_limits_lower[i] 
                - self.joint_limit_tolerance
            )
        
        self.kinematic_tree.set_joint_limits_upper(global_joint_limits_upper)
        self.kinematic_tree.set_joint_limits_lower(global_joint_limits_lower)


    def lookup_base_pose(self): #Finds the current tf transform for the base and converts it to XYZ+RPY
        base_transform = self.tfBuffer.lookup_transform(
            "odom","base",rospy.Time(0),rospy.Duration(3.0)
        )
        base_pose = array([0.0]*6)

        rpy = transformations.euler_from_quaternion(
            [
            base_transform.transform.rotation.x,
            base_transform.transform.rotation.y,
            base_transform.transform.rotation.z,
            base_transform.transform.rotation.w,
            ]
        )
        base_pose[0] = base_transform.transform.translation.x
        base_pose[1] = base_transform.transform.translation.y
        base_pose[2] = base_transform.transform.translation.z
        base_pose[3] = rpy[0]
        base_pose[4] = rpy[1]
        base_pose[5] = rpy[2]
        
        return base_pose
    
    def init_robot_pose(self): #Initializes the robot pose in EXOTica
        self.q = self.problem.start_state.copy()
        if (len(self.joint_names) - self.base_dof) != 0:
            self.joint_states_callback(
                #A Joinst State message with 7 joints is published to the below topic
                rospy.wait_for_message("/z1_gazebo/joint_states", JointState) 
            )                                                                  
        
        if self.base_dof != 0:
            self.q[0 : self.base_dof] = self.lookup_base_pose()
            joint_limits = self.kinematic_tree.get_joint_limits()
            #Clamp q to within constraints
            for i in range(0, self.base_dof) :
                if self.q[i] > joint_limits[i,1]:
                    self.q[i] = joint_limits[i,1]
                elif self.q[i] < joint_limits[i,0]:
                    self.q[i] = joint_limits[i,0]
        
        self.scene.set_model_state(self.q[: self.scene.num_positions])
        self.problem.update(self.scene.get_controlled_state(),0)
        self.scene.get_kinematic_tree().publish_frames()


    def joint_states_callback(self, joint_states):
        #Note: assumes all joint_states messages contain all joints
        for i in range(0,len(self.joint_names)):
            if self.joint_names[i] in joint_states.name:
                self.q[i] = joint_states.position[
                    joint_states.name.index(self.joint_names[i])
                ]
    
    def get_error(self): #Computes the task error as a scalar value
        error_array = array(self.problem.get_state_cost(0))
        square_error = error_array.dot(error_array)
        #rospy.loginfo("Pose error %f", math.sqrt(square_error))
        return math.sqrt(square_error)
    
    def interactive_servoing(self):
        update_xy = self.lookup_base_pose()
        self.scene.set_model_state_map(
            {
                "world_joint/trans_x" : update_xy[0],
                "world_joint/trans_y" : update_xy[1],
            }
        )

        self.q[0:2] = update_xy[0:2]
        joint_limits = self.kinematic_tree.get_joint_limits()
        global_joint_limits_upper = joint_limits[:, 1]
        global_joint_limits_lower = joint_limits[:, 0]

        global_joint_limits_upper[0] = (
                update_xy[0]
                + self.relative_joint_limits_upper[0]
                + self.joint_limit_tolerance
            )  # x
        global_joint_limits_upper[1] = (
                update_xy[1]
                + self.relative_joint_limits_upper[1]
                + self.joint_limit_tolerance
            )  # y

        global_joint_limits_lower[0] = (
                update_xy[0]
                + self.relative_joint_limits_lower[0]
                - self.joint_limit_tolerance
            )  # x
        global_joint_limits_lower[1] = (
                update_xy[1]
                + self.relative_joint_limits_lower[1]
                - self.joint_limit_tolerance
            )  # y
        
        self.kinematic_tree.set_joint_limits_upper(global_joint_limits_upper)
        self.kinematic_tree.set_joint_limits_lower(global_joint_limits_lower)

    def publish_to_robot(self): #Publishes motion plan as a single waypoint trajectory_msgs/JointTrajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = self.q[0:12].copy()
        trajectory_point.time_from_start = rospy.Duration.from_sec(self.dt)

        trajectory_msg.points.append(trajectory_point)
        self.motion_plan_publisher.publish(trajectory_msg)
    
    def iterate(self): #Doesn't update q from the base pose, therefore exotica robot state is not updated

        #Inform exotica of unexpected changes to XY as a side-effect of ANYNova rotating its base
        self.interactive_servoing()

        self.problem.start_time = self.t #Set the problem start time
        self.problem.start_state = self.q #Set the problem start state

        #Solve using MPC
        solution = self.solver.solve()
        

        self.problem.update(self.q,solution[0],0) 
        print("Exotica Solution:")
        print(solution[0])

        #Update the joint positions
        self.q[:] = self.problem.X[:,1].copy()
        self.scene.set_model_state(self.q[: self.scene.num_positions])

    def perform_motion(self):       #Trajectory goal defined as a target pose (not timed)
        self._result.result = True
        self.init_robot_pose()
        self.t = 0.0
        counter = 0

        while (counter < self.counter) and (self.t < self.t_limit):
            try:
                #Check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    self._result.result = False
                    rospy.logwarn("%s: PREEMPTED (Goal Cancelled)",self.action_name)
                    self._as.set_preempted()
                    break

                self.iterate() #Generate motion plan via EXOTica
                self.publish_to_robot() #Send motion plan to robot
                error = self.get_error() #Calculate error

                #Publish feedback to client
                self._feedback.error = error
                self._as.publish_feedback(self._feedback)

                #Check whether to increment the counter
                if error < self.tolerance:
                    counter += 1
                
                #Sleep and increment time
                self.rate.sleep()
                self.t = self.t + self.dt
                #rospy.loginfo("t: %s", str(self.t))

            #Check that the motion plan hasn't been aborted via keyboard interupt
            except KeyboardInterrupt:
                self._result.result = False
                rospy.logwarn("%s: ABORTED (KeyboardInterrupt)" ,self.action_name)
                self._as.set_aborted(self._result)
                break

        #Check that EEF has reached target
        if self._result.result == True:
            error = self.get_error()
            if error < self.tolerance:
                rospy.loginfo("%s: SUCCESS (Approached Target)", self.action_name)
                self._as.set_succeeded(self._result)
            else:
                self._result.result = False
                rospy.logwarn("%s: ABORTED (Did not reached target within time limit)"
                              , self.action_name)
                self._as.set_aborted(self._result)
        
        self.problem.start_time = 0 #Reset problem start time

    def perform_trajectory(
        self, trajectory
    ):  # Trajectory goal defined as a set of time-indexed waypoints
        self._result.result = True
        self.init_robot_pose()
        self.scene.add_trajectory_from_array("TargetRelative", trajectory, 1.0)
        self.t = 0.0
        # self_counter = 0 # Counter for how many consecutive iterations the real EEF has been beyond the error margin for the motion plan EEF
        t_limit = trajectory[-1, 0]

        # Check that EEF is within tolerance of the start waypoint
        self.problem.start_time = self.t  # Set problem start time
        self.iterate()  # Generate motion plan via EXOTica
        error = self.get_error()  # Calculate error
        if error > self.start_tolerance:
            self._result.result = False
            rospy.loginfo(
                "%s: ABORTED (EEF Start Pose Beyond Tolerance)" % self.action_name
            )
            self._as.set_aborted(self._result)

        else:
            self.init_robot_pose()  # Reset pose after checking start tolerance
            while self.t < t_limit:
                try:
                    # Check that preempt has not been requested by the client
                    if self._as.is_preempt_requested():
                        self._result.result = False
                        rospy.logwarn(
                            "%s: PREEMPTED (Goal Cancelled)" % self.action_name
                        )
                        self._as.set_preempted()
                        break

                    self.iterate()  # Generate motion plan via EXOTica
                    self.publish_to_robot()  # Send motion plan to robot
                    error = self.get_error()  # Calculate error

                    # Publish feedback to Client
                    self._feedback.error = error
                    self._as.publish_feedback(self._feedback)

                    # Sleep and increment time
                    self.rate.sleep()
                    self.t = self.t + self.dt

                # Check that motion plan has not been aborted via KeyboardInterrupt
                except KeyboardInterrupt:
                    self._result.result = False
                    rospy.logwarn("%s: ABORTED (KeyboardInterrupt)" % self.action_name)
                    self._as.set_aborted(self._result)
                    break

            # Check that EEF has reached Final Waypoint
            rospy.loginfo(self._result.result)
            if self._result.result == True:
                error = self.get_error()
                if error < self.tolerance:
                    rospy.loginfo(
                        "%s: SUCCESS (Completed Trajectory)" % self.action_name
                    )
                    self._as.set_succeeded(self._result)
                else:
                    self._result.result = False
                    rospy.logwarn(
                        "%s: ABORTED (Did Not Reach Final Waypoint Within Time Limit)"
                        % self.action_name
                    )
                    self._as.set_aborted(self._result)

        # Reset problem start time and remove trajectory
        self.problem.start_time = 0
        self.scene.remove_trajectory("TargetRelative")

    def get_frame_from_pose(self, pose_):  # Important for defining target frames for trajectories
        frame_ = array([0.0] * 7)
        frame_[0] = pose_.position.x
        frame_[1] = pose_.position.y
        frame_[2] = pose_.position.z
        frame_[3] = pose_.orientation.x
        frame_[4] = pose_.orientation.y
        frame_[5] = pose_.orientation.z
        frame_[6] = pose_.orientation.w
        return frame_
    
    



        









        
        