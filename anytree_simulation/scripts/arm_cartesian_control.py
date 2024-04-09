#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

'''
This program allows the user to input coordinates for arm to go to,
It handles communications with the arm through ros
The coordinates should be defined relative to robots base

'''
rospy.init_node('arm_jointcontrol_node')
publisher = rospy.Publisher('arm_pose_command',JointTrajectoryPoint,queue_size=1)
message = JointTrajectoryPoint()

while not rospy.is_shutdown():
    coordinates = []
    print("Input the coordinates in 'roll pitch yaw x y z' form in the following line\n ")
    input_str = input("Enter floating numbers separated by commas: ")
    str_values = input_str.split(',')
    valid_coordinates = True
    for value in str_values:
        try:
            #Attem to convert string to a float
            double_value = float(value.strip())
            coordinates.append(double_value)
        except ValueError:
            #If conversion fails, it's not a valid double
            valid_coordinates = False
            print("Entered value is not a valid double")
            break
    if valid_coordinates:
        if (len(coordinates) != 6):
            print("There must be 6 values, please try again")
        else:
            message.positions = coordinates
            publisher.publish(message)
            
    else:
        print('All entered values must be doubles, try again')
        continue
