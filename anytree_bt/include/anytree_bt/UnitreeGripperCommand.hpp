#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <anytree_bt/bt_action_client_node.hpp>
#include <ros/ros.h>
#include <anytree_msgs/gripperCommandAction.h>
#include <anytree_msgs/gripperCommandGoal.h>


// Exists as a rename because the library in behavior_tree already has a GripperCommand node
class UnitreeGripperCommand : public ActionClientNode {
    
    typedef actionlib::SimpleActionClient<
    anytree_msgs::gripperCommandAction> Client;

public:
    UnitreeGripperCommand(const std::string &name, const BT::NodeConfig &config) :
    ActionClientNode(name,config) {
        client_ptr = std::unique_ptr<Client>(
            new Client("gripperCommand_as",true)
        );
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("targetQ")};
    }

    void sendStartRequest() override {
        client_ptr->waitForServer();
        anytree_msgs::gripperCommandGoal goal;

        auto targetQ_bt = getInput<double>("targetQ");

        if (!targetQ_bt) {
            throw BT::RuntimeError("Missing required inputs");
        }

        goal.targetQ = targetQ_bt.value();
        ROS_INFO("Sending gripper goal");

        client_ptr->sendGoal(goal);
    }

    bool getResult() override {
        anytree_msgs::gripperCommandResultConstPtr result;
        result = client_ptr->getResult();
        return result->result;
    }

     State getState() override {
        auto state = client_ptr->getState();
        if (state == State::StateEnum::ABORTED) {
        (void)getResult();
        }
        return state;
    }

    void sendAbortSignal() override { client_ptr->cancelGoal(); }

private:
    std::unique_ptr<Client> client_ptr;
};