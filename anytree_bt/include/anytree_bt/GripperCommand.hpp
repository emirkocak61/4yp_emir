#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


class GripperCommand : public BT::SyncActionNode {
public:
    GripperCommand(const std::string& name, const BT::NodeConfig& config) : 
        BT::SyncActionNode(name, config) {
            pub = nh_.advertise<std_msgs::Bool>("/z1_gazebo/gripper_command",10);
        }
    
    ~GripperCommand() override {}
    
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("command")};
    }
    BT::NodeStatus tick() override {
        ROS_INFO("Sending gripper goal");
        auto gripper_command = getInput<std::string>("command");
        std_msgs::Bool gripper_msg;
        if (gripper_command.value() == "open") {gripper_msg.data = true;}
        //If not open it probably wants to clos the gripper
        else {gripper_msg.data = false;}
        pub.publish(gripper_msg);
        return BT::NodeStatus::SUCCESS;
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub;
};
