#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
from anytree_msgs.srv import selectStrategy, selectStrategyResponse
from std_msgs.msg import Float64
import pickle
import rospkg
import os

np.set_printoptions(precision=3)

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path("anytree_simulation")


class selectStrategyServer:
    def __init__(self):

        self.device_state = 0.0

        self.initialise_manipulation_database()

        rospy.Subscriber("device_state", Float64, self.updateDeviceState)

        # Dict of dicts, the strategies for each device type generally ordered from lowest-to-highest strength

        # Strategy effort limits (Nm) [j1 ,j2, j3, j4, j5 ,j6]
        self.strategy_effort_limits = {
            "needle_valve": {
                0: [100.0, 100.0, 100.0, 100.0, 100.0, 5.0],
                1: [100.0, 100.0, 100.0, 100.0, 100.0, 10.0],
            },
            "button": {
                0: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            },
            "DN40_globe_valve": {
                0: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            },
        }

        # Device effort limits (Nm) [j1 ,j2, j3, j4, j5 ,j6]
        self.device_effort_limits = {
            "needle_valve": {
                0: [100.0, 100.0, 100.0, 100.0, 100.0, 8.0],
                1: [100.0, 100.0, 100.0, 100.0, 100.0, 8.0],
            },
            "button": {
                0: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            },
            "DN40_globe_valve": {
                0: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            },
        }

        # Strategy angle limits (e.g. rad) [min, max, rot_sym]
        self.strategy_angle_limits = {
            "needle_valve": {
                0: [-3.6652, 0.5236, 3.1415],
                1: [-1.5707, 1.5708, 3.1415],
            },
            "button": {
                0: [-1.0000, 1.0000, 6.2831],
            },
            "DN40_globe_valve": {
                0: [-0.5236, 1.5708, 2.0944],
            },
        }

        # Strategy speed params [manipulation_rate (e.g. rad per iteration), grasp_time (s)]
        self.strategy_speed_params = {
            "needle_valve": {
                0: [0.005, 9.0],
                1: [0.002, 20.0],
            },
            "button": {
                0: [0.25, 9.0],
            },
            "DN40_globe_valve": {
                0: [0.25, 9.0],
            },
        }

        self.srv = rospy.Service(
            "selectStrategy",
            selectStrategy,
            self.handle_selectStrategy,
        )

    def handle_selectStrategy(self, req):

        self.device_type = req.device_type
        self.device_id = req.device_id
        self.input_strategy = req.input_strategy
        self.manipulation_todo = req.manipulation_todo
        self.direction = req.direction

        with open(
            PACKAGE_PATH
            + "/manipulation_database/manipulation_database.pkl",
            "rb",
        ) as f:
            manipulation_database = pickle.load(f)

        if self.device_type not in self.strategy_effort_limits:
            # Error - unknown device
            rospy.logerr("selectStrategy: Unknown Device (%s)", self.device_type)
            response = self.error_case()
            return response

        elif self.device_type not in manipulation_database:
            if self.input_strategy != -1:
                if self.input_strategy not in self.strategy_effort_limits[self.device_type]:
                    rospy.logerr("selectStrategy: Unknown Strategy (%d) Requested", self.input_strategy)
                    response = self.error_case()
                    return response
                else:
                    selected_strategy = self.input_strategy
            else:
                # Choose fastest strategy
                estimated_durations = self.estimate_duration()
                strategy_indices_sorted = [i for i, x in sorted(enumerate(estimated_durations), key=lambda x: x[1])]
                selected_strategy = strategy_indices_sorted[0]

        elif self.device_id not in manipulation_database[self.device_type]:
            if self.input_strategy != -1:
                if self.input_strategy not in self.strategy_effort_limits[self.device_type]:
                    rospy.logerr("selectStrategy: Unknown Strategy (%d) Requested", self.input_strategy)
                    response = self.error_case()
                    return response
                else:
                    selected_strategy = self.input_strategy
            else:
                # Choose fastest strategy
                estimated_durations = self.estimate_duration()
                strategy_indices_sorted = [i for i, x in sorted(enumerate(estimated_durations), key=lambda x: x[1])]
                selected_strategy = strategy_indices_sorted[0]

        else:
            # Choose fastest strategy that permits efforts for device in manipulation data

            max_effort = np.array(
                [
                    manipulation_database[self.device_type][self.device_id][
                        "j1_effort"
                    ]
                    .abs()
                    .max(),
                    manipulation_database[self.device_type][self.device_id][
                        "j2_effort"
                    ]
                    .abs()
                    .max(),
                    manipulation_database[self.device_type][self.device_id][
                        "j3_effort"
                    ]
                    .abs()
                    .max(),
                    manipulation_database[self.device_type][self.device_id][
                        "j4_effort"
                    ]
                    .abs()
                    .max(),
                    manipulation_database[self.device_type][self.device_id][
                        "j5_effort"
                    ]
                    .abs()
                    .max(),
                    manipulation_database[self.device_type][self.device_id][
                        "j6_effort"
                    ]
                    .abs()
                    .max(),
                ]
            )
            max_effort[np.isnan(max_effort)] = 0.0
            rospy.loginfo("Manipulation Data Max Efforts: \n [%f, %f, %f, %f, %f, %f]", max_effort[0], max_effort[1], max_effort[2], max_effort[3], max_effort[4], max_effort[5])
            # If input_strategy == -1 (or "" in the BT interface), selectStrategy will attempt to select the fastest feasible strategy
            # If input_strategy != -1, selectStrategy will simply pass it through to the output (if it exists and is feasible)        
            if self.input_strategy != -1:
                if self.input_strategy not in self.strategy_effort_limits[self.device_type]:
                    rospy.logerr("selectStrategy: Unknown Strategy (%d) Requested", self.input_strategy)
                    response = self.error_case()
                    return response
                elif all(
                    self.strategy_effort_limits[self.device_type][self.input_strategy] >= max_effort
                ):
                    selected_strategy = self.input_strategy
                else:
                    rospy.logerr("selectStrategy: Requested Strategy (%d) Does Not Permit Efforts in Manipulation Data", self.input_strategy)
                    response = self.error_case()
                    return response
            else:
                selected_strategy = self.input_strategy # -1
                estimated_durations = self.estimate_duration()
                strategy_indices_sorted = [i for i, x in sorted(enumerate(estimated_durations), key=lambda x: x[1])]
                # Iterate through all strategies from fastest to slowest until a feasible strategy is found
                for trial_strategy in strategy_indices_sorted:
                    if all(
                        self.strategy_effort_limits[self.device_type][trial_strategy] >= max_effort
                    ):
                        selected_strategy = trial_strategy
                        break
            if selected_strategy == -1:
                rospy.logwarn(
                    "selectStrategy: Manipulation Data Contains Efforts Not Permitted by any Known Strategies"
                )
                response = self.error_case()
                return response
        
        rospy.loginfo("Selected Strategy: %d", selected_strategy)
        response = selectStrategyResponse()
        response.selected_strategy = selected_strategy
        response.strategy_effort_limit = self.strategy_effort_limits[self.device_type][selected_strategy]
        response.min_angle = self.strategy_angle_limits[self.device_type][selected_strategy][0]
        response.max_angle = self.strategy_angle_limits[self.device_type][selected_strategy][1]
        response.rot_sym_angle = self.strategy_angle_limits[self.device_type][selected_strategy][2]
        response.device_effort_limit = self.device_effort_limits[self.device_type][selected_strategy]
        return response

    def error_case(self):
        response = selectStrategyResponse()
        response.selected_strategy = -1
        response.strategy_effort_limit = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        response.min_angle = 0.0
        response.max_angle = 0.0
        response.rot_sym_angle = 0.0
        response.device_effort_limit = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return response
    
    def initialise_manipulation_database(self):
        if not os.path.exists(PACKAGE_PATH + "/manipulation_database/manipulation_database.pkl"):
            manipulation_database = {}
            with open(
                PACKAGE_PATH
                + "/manipulation_database/manipulation_database.pkl",
                "wb",
            ) as g:
                pickle.dump(manipulation_database, g, protocol=pickle.HIGHEST_PROTOCOL)

    def updateDeviceState(self, device_state_msg):
        self.device_state = device_state_msg.data

    def estimate_duration(self):
        dt = 0.02
        estimated_durations = []
        for strategy in self.strategy_speed_params[self.device_type]:
            t = self.strategy_speed_params[self.device_type][strategy][1] # always start with a grasp
            manipulation_done = 0.0
            device_state = self.device_state

            while manipulation_done < self.manipulation_todo:
                # Regrasp (smart twisting)
                if device_state < self.strategy_angle_limits[self.device_type][strategy][0]:
                    device_state += self.strategy_angle_limits[self.device_type][strategy][2]
                    t += self.strategy_speed_params[self.device_type][strategy][1]

                if device_state > self.strategy_angle_limits[self.device_type][strategy][1]:
                    device_state -= self.strategy_angle_limits[self.device_type][strategy][2]
                    t += self.strategy_speed_params[self.device_type][strategy][1]
                # Twist
                manipulation_done += self.strategy_speed_params[self.device_type][strategy][0]
                device_state += self.strategy_speed_params[self.device_type][strategy][0] * self.direction
                t += dt

            estimated_durations.append(t)
            rospy.loginfo("Strategy: %d | Est. Duration (s): %f", strategy, t)
        return estimated_durations

if __name__ == "__main__":

    rospy.init_node("selectStrategy_server")

    s = selectStrategyServer()
    rospy.loginfo("Strategy Selector ready")

    rospy.spin()
