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
package_path = rp.get_path("motion_planning_exotica")

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
        self.t_limit = 90.0 #time limit

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
            "{motion_planning_exotica}/resources/configs"
            + str(self.action_name)
            + "_mpc.xml"
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
        self.joint_limits_tolerance = 0.0001
        self.set_joint_limits()

        joint_velocities = self.kinematic_tree.get_velocity_limits()

        #Create a subscriber to allow external nodes to trigger a reset of robot's rest pose
        rospy.Subscriber("/reset_rest_pose",Bool,self.reset_rest_pose_callback)
    
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
                + self.joint_limits_tolerance
            )
            global_joint_limits_lower[i] = (
                self.rest_pose[i] 
                + self.relative_joint_limits_lower[i] 
                - self.joint_limits_tolerance
            )
        
        self.kinematic_tree.set_joint_limits_upper(global_joint_limits_upper)
        self.kinematic_tree.set_joint_limtis_lower(global_joint_limits_lower)


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




        
        