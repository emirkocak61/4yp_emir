#!/usr/bin/env python3

import pyexotica as exo
from pyexotica.publish_trajectory import sig_int_handler
import signal
import rospy
from actionlib import SimpleActionServer
from bt_drs_msgs import (
    approachTargetAction,
    approachTargetFeedback,
    approachTargetResult,
)

from motion_planner_py import MPCMotionPlannerBaseClass

class approachTargetActionServer(MPCMotionPlannerBaseClass):
    
    _feedback = approachTargetFeedback()
    _result = approachTargetResult()

    def __init__(self):
        self.robot_name = "anytree"
        self.action_name = "approachTarget"
        super().__init__()
        self._as = SimpleActionServer(
            str(self.action_name + "_as"),
            approachTargetAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
    
    def execute_cb(self,goal):
        #Define target frame
        T_approach = exo.KDLFrame(self.get_frame_from_pose(goal.target))

        #Attach target frame in absolute world frame
        self.scene.attach_object_local("Target","",T_approach)

        rospy.loginfo("Approaching target")
        self.perform_motion()

if __name__ == "__main__":

    exo.Setup.init_ros("approach_target_mpc")
    rospy.init_node("approach_target_mpc")

    s = approachTargetActionServer()
    signal.signal(signal.SIGINT,sig_int_handler)

    rospy.spin()
