#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


class ResetArmPose : public BT::SyncActionNode {
public:
    ResetArmPose(const std::string& name) : 
        BT::SyncActionNode(name, {}) {
            pub = nh_.advertise<std_msgs::Bool>("/reset_arm_pose",10);
        }
    
    ~ResetArmPose() override {}
       
    BT::NodeStatus tick() override {
        std_msgs::Bool reset_arm_pose;
        reset_arm_pose.data = true;
        pub.publish(reset_arm_pose);
        return BT::NodeStatus::SUCCESS;
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub;
};
