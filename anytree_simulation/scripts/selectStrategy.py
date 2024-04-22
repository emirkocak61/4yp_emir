#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
from anytree_msgs.srv import selectStrategy, selectStrategyResponse
import pickle
import rospkg
import os

np.set_printoptions(precision=3)

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path("anytree_simulation")


class selectStrategyServer:
    def __init__(self):

        self.initialise_manipulation_database()

        # Dict of dicts, the strategies for each device type generally ordered from lowest-to-highest strength
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

        self.strategy_angle_limits = {
            "needle_valve": {
                0: [-3.6652, 0.5236, 3.1415],
                1: [-1.5707, 1.5708, 3.1415],
            },
            "button": {
                0: [-1.0000, 1.0000, 1.0000],
            },
            "DN40_globe_valve": {
                0: [-0.5236, 1.5708, 2.0944],
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
                # Choose lowest-F/T strategy
                selected_strategy = next(iter(self.strategy_effort_limits[self.device_type]))

        elif self.device_id not in manipulation_database[self.device_type]:
            if self.input_strategy != -1:
                if self.input_strategy not in self.strategy_effort_limits[self.device_type]:
                    rospy.logerr("selectStrategy: Unknown Strategy (%d) Requested", self.input_strategy)
                    response = self.error_case()
                    return response
                else:
                    selected_strategy = self.input_strategy
            else:
                # Choose lowest-F/T strategy
                selected_strategy = next(iter(self.strategy_effort_limits[self.device_type]))

        else:
            # Choose lowest-strength strategy that permits measured efforts for device

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
            # If input_strategy == -1 (or "" in the BT interface), selectStrategy will attempt to select a feasible strategy
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
                for trial_strategy in self.strategy_effort_limits[self.device_type]:
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


if __name__ == "__main__":

    rospy.init_node("selectStrategy_server")

    s = selectStrategyServer()
    rospy.loginfo("Strategy Selector ready")

    rospy.spin()
