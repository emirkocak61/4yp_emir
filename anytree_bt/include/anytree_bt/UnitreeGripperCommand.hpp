#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// Exists as a rename because the library in behavior_tree already has a GripperCommand node
class UnitreeGripperCommand : public BT::SyncActionNode {
public:
    UnitreeGripperCommand(const std::string& name, const BT::NodeConfig& config) : 
        BT::SyncActionNode(name, config) {
            pub = nh_.advertise<std_msgs::Bool>("/z1_gazebo/gripper_command",10);
        }
    
    ~UnitreeGripperCommand() override {}
    
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
        while (pub.getNumSubscribers() < 1) {
        }
        pub.publish(gripper_msg);
        ros::Duration(1.0).sleep();
        return BT::NodeStatus::SUCCESS;
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub;
};
