#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose
from bt_drs_msgs.msg import (
    approachTargetAction,
    approachTargetGoal,
)
from tf.transformations import quaternion_from_euler
def feedback_callback(feedback):
    #Log feedback to the console
    rospy.loginfo("Current eror: %s" % str(feedback.error))
def main():
    # Initialize the ROS node
    rospy.init_node('send_approach_target')

    # Wait for the parameter to be available on the parameter server
    while not rospy.has_param("target_pose_euler"):
        rospy.sleep(0.1)

    # Get the target pose from the ROS parameter server as xyz + ypr (yaw, pitch, roll)
    target_pose_euler = rospy.get_param("target_pose_euler")
    
    # Convert Euler angles (Y, P, R) to quaternion
    target_pose_q = quaternion_from_euler(target_pose_euler['R'], target_pose_euler['P'], target_pose_euler['Y'])

    # Create the goal Pose
    goal = approachTargetGoal()
    goal.target.position.x = target_pose_euler['x']
    goal.target.position.y = target_pose_euler['y']
    goal.target.position.z = target_pose_euler['z']
    goal.target.orientation.x = target_pose_q[0]
    goal.target.orientation.y = target_pose_q[1]
    goal.target.orientation.z = target_pose_q[2]
    goal.target.orientation.w = target_pose_q[3]

    # Create a SimpleActionClient and send the goal (assuming the action server is set up)
    client = SimpleActionClient('approachTarget_as', approachTargetAction)
    client.wait_for_server()
    #Send goal
    client.send_goal(goal,feedback_cb=feedback_callback)
    #Wait for server to finish action
    client.wait_for_result()
    #Get result
    result = client.get_result()
    if result.result:
        rospy.loginfo("Action successful: approached target")
    else:
        rospy.loginfo("Action failed...")

if __name__ == "__main__":
    main()
    rospy.spin()
