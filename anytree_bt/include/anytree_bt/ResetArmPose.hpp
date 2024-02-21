#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


class ResetArmPose : public BT::SyncActionNode {
public:
    ResetArmPose(const std::string& name, const BT::NodeConfig& config) : 
        BT::SyncActionNode(name, config) {
            pub = nh_.advertise<std_msgs::Bool>("/z1_gazebo/reset_arm_pose",10);
        }
    
    ~ResetArmPose() override {}

    static BT::PortsList providedPorts() {
        return {};
    }
    

    BT::NodeStatus tick() override {
        ROS_INFO("Resetting arm pose");
        std_msgs::Bool arm_command;
        arm_command.data = true;
        //If not open it probably wants to close the gripper
        pub.publish(arm_command);
        ros::Duration(1.0).sleep();
        return BT::NodeStatus::SUCCESS;
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub;
};
