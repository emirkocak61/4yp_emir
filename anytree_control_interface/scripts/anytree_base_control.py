#!/usr/bin/env python3

import pyexotica as exo
import math
from numpy import array
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ANYTreeBaseControlInterface:

    def __init__(self):
        
        rospy.init_node("anytree_control_interface")

        self.base_command_publisher = rospy.Publisher(
            "/motion_reference/command_pose",
            PoseStamped,
            queue_size=10,
            tcp_nodelay=True,
        )

        rospy.Subscriber("/motion_plan", JointTrajectory, self.compute_pose_stamped)
        rospy.Subscriber("/clear_base_command", Bool, self.clear_base_command)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        w_T_base_at_rest = self.tfBuffer.lookup_transform(
            "odom","base",rospy.Time(0),rospy.Duration(1.0)
        )

        self.w_T_base_at_rest_KDL = exo.KDLFrame(
            [
                w_T_base_at_rest.transform.translation.x,
                w_T_base_at_rest.transform.translation.y,
                w_T_base_at_rest.transform.translation.z,
                w_T_base_at_rest.transform.rotation.x,
                w_T_base_at_rest.transform.rotation.y,
                w_T_base_at_rest.transform.rotation.z,
                w_T_base_at_rest.transform.rotation.w,
            ]
        )

        rospy.Subscriber("/reset_rest_pose", Bool, self.reset_rest_transform_callback)

        self.hold_position = False
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = "base"

        self.timer = rospy.Timer(rospy.Duration(0.02),self.timer_callback)

    #Determines the base pose (relative to 'base origin') from XYZ + RPY
    def compute_pose_stamped(self, trajectory_msg):
        trajectory_point = trajectory_msg.points[0]
        w_T_base_target_KDL = exo.KDLFrame(
            [
                trajectory_point.positions[0],
                trajectory_point.positions[1],
                trajectory_point.positions[2],
                trajectory_point.positions[3],
                trajectory_point.positions[4],
                trajectory_point.positions[5],
            ]
        )
    
        base_at_rest_T_base_target_KDL = (
                self.w_T_base_at_rest_KDL.inverse() * w_T_base_target_KDL
            )
        
        target_vector = base_at_rest_T_base_target_KDL.get_translation_and_quaternion()

        # The ANYmal controller inverts pitch and roll, so we need to pre-invert them...

        euler = euler_from_quaternion(target_vector[3:])

        new_quarternion = quaternion_from_euler(-euler[0], -euler[1], euler[2])

        self.pose_stamped.pose.orientation.x = new_quarternion[0]
        self.pose_stamped.pose.orientation.y = new_quarternion[1]
        self.pose_stamped.pose.orientation.z = new_quarternion[2]
        self.pose_stamped.pose.orientation.w = new_quarternion[3]

        self.pose_stamped.pose.position.x = 0.0
        self.pose_stamped.pose.position.y = 0.0
        self.pose_stamped.pose.position.z = target_vector[2]

        self.hold_position = True

    def timer_callback(self, event):
        if self.hold_position:
            self.pose_stamped.header.stamp = rospy.Time.now()
            self.base_command_publisher.publish(self.pose_stamped)  # Publish to cmd_vel

    def clear_base_command(self, clear_command):
        if clear_command.data:
            self.hold_position = False

    def reset_rest_transform_callback(self, reset_rest_pose):
        if reset_rest_pose.data:
            w_T_base_at_rest = self.tfBuffer.lookup_transform(
                "odom", "base", rospy.Time(0), rospy.Duration(1.0)
            )
            self.w_T_base_at_rest_KDL = exo.KDLFrame(
                [
                    w_T_base_at_rest.transform.translation.x,
                    w_T_base_at_rest.transform.translation.y,
                    w_T_base_at_rest.transform.translation.z,
                    w_T_base_at_rest.transform.rotation.x,
                    w_T_base_at_rest.transform.rotation.y,
                    w_T_base_at_rest.transform.rotation.z,
                    w_T_base_at_rest.transform.rotation.w,
                ]
            )

if __name__ == "__main__":

    controller = ANYTreeBaseControlInterface() #Instantiate the controller
    rospy.spin()
    controller.timer.shutdown()





