#!/usr/bin/env python3

import rospy
from bt_drs_msgs.srv import recordManipulationData, recordManipulationDataResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import pickle
import rospkg
import pandas as pd
import os

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path("anytree_simulation")


class ManipulationDataRecorder:
    def __init__(self):

        self.record_data = False
        self.device_state = 0.0

        self.initialise_manipulation_database()

        rospy.Subscriber("z1_gazebo/joint_states_filtered", JointState, self.updateEffortData)
        rospy.Subscriber("device_state", Float64, self.updateDeviceState)

        self.srv = rospy.Service(
            "recordManipulationData",
            recordManipulationData,
            self.handle_recordManipulationData,
        )

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def handle_recordManipulationData(self, req):
        self.device_type = req.device_type
        self.device_id = req.device_id
        self.direction = req.direction
        if req.record_data and not self.record_data:
            self.start_time = rospy.Time.now()
            self.record_data = req.record_data
            self.new_data = [
                pd.DataFrame(
                    columns=[
                        "direction",
                        "duration",
                        "device_state",
                        "j1_effort",
                        "j2_effort",
                        "j3_effort",
                        "j4_effort",
                        "j5_effort",
                        "j6_effort",
                    ]
                )
            ]
            rospy.loginfo("Manipulation Data Recorder ON")
        elif self.record_data and not req.record_data:
            self.record_data = req.record_data
            with open(
                PACKAGE_PATH
                + "/manipulation_database/manipulation_database.pkl",
                "rb",
            ) as f:
                manipulation_database = pickle.load(f)

            manipulation_database = self.add_new_data(manipulation_database)

            with open(
                PACKAGE_PATH
                + "/manipulation_database/manipulation_database.pkl",
                "wb",
            ) as g:
                pickle.dump(
                    manipulation_database, g, protocol=pickle.HIGHEST_PROTOCOL
                )
            rospy.loginfo("Manipulation Data Recorder OFF")
        
        return recordManipulationDataResponse(True)

    def updateEffortData(self, joint_states_msg):
        self.effort_data = joint_states_msg.effort

    def updateDeviceState(self, device_state_msg):
        self.device_state = device_state_msg.data

    def timer_callback(self, event):
        if self.record_data:
            self.new_data.append(
                pd.DataFrame(
                    {
                        "direction": self.direction,
                        "duration": rospy.Time.now() - self.start_time,
                        "device_state": self.device_state,
                        "j1_effort": self.effort_data[0],
                        "j2_effort": self.effort_data[1],
                        "j3_effort": self.effort_data[2],
                        "j4_effort": self.effort_data[3],
                        "j5_effort": self.effort_data[4],
                        "j6_effort": self.effort_data[5],
                    },
                    index=[self.start_time],
                )
            )

    def add_new_data(self, manipulation_database):
        if self.device_type not in manipulation_database:
            manipulation_database[self.device_type] = {}
        if self.device_id not in manipulation_database[self.device_type]:
            manipulation_database[self.device_type][
                self.device_id
            ] = pd.DataFrame(
                columns=[
                    "direction",
                    "duration",
                    "device_state",
                    "j1_effort",
                    "j2_effort",
                    "j3_effort",
                    "j4_effort",
                    "j5_effort",
                    "j6_effort",
                ]
            )
        manipulation_database[self.device_type][self.device_id] = pd.concat(
            [
                manipulation_database[self.device_type][self.device_id],
                pd.concat(self.new_data),
            ]
        )
        return manipulation_database
    
    def initialise_manipulation_database(self):
        if not os.path.exists(PACKAGE_PATH + "/manipulation_database/manipulation_database.pkl"):
            manipulation_database = {}
            with open(
                PACKAGE_PATH
                + "/manipulation_database/manipulation_database.pkl",
                "wb",
            ) as g:
                pickle.dump(manipulation_database, g, protocol=pickle.HIGHEST_PROTOCOL)


if __name__ == "__main__":

    rospy.init_node("ManipulationDataRecorderNode")

    s = ManipulationDataRecorder()
    rospy.loginfo("Manipulation Data Recorder ready")

    rospy.spin()
    s.timer.shutdown()
