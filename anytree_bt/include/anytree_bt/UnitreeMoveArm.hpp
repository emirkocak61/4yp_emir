#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <anytree_bt/bt_action_client_node.hpp>
#include <anytree_bt/custom_types.hpp>
#include <ros/ros.h>
#include <anytree_msgs/moveArmAction.h>
#include <anytree_msgs/moveArmGoal.h>

class UnitreeMoveArm : public ActionClientNode {
    
    typedef actionlib::SimpleActionClient<
    anytree_msgs::moveArmAction> Client;
public:
    UnitreeMoveArm(const std::string &name, const BT::NodeConfig &config) :
    ActionClientNode(name,config) {
        client_ptr = std::unique_ptr<Client>(
            new Client("/z1_gazebo/moveArm_as",true)
        );
    }   

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("label"),
                BT::InputPort<double>("duration"),
                BT::InputPort<Position3D>("position"),
                BT::InputPort<Orientation3D>("orientation")};
    }

    void sendStartRequest() override {
        client_ptr->waitForServer();
        anytree_msgs::moveArmGoal goal;

        auto label_bt = getInput<std::string>("label");
        auto duration_bt = getInput<double>("duration");
        auto position_bt = getInput<Position3D>("position");
        auto orientation_bt = getInput<Orientation3D>("orientation");


        goal.label = label_bt.value();
        
        if (position_bt && orientation_bt) {
            std::vector<double> posture = {orientation_bt.value().R,
                                            orientation_bt.value().P,
                                            orientation_bt.value().Y,
                                            position_bt.value().x,
                                            position_bt.value().y,
                                            position_bt.value().z};

            goal.posture = posture;
        }
        
        
        goal.duration = duration_bt.value();
        
        std::cout << "Sending arm command..." << std::endl;
        
        client_ptr->sendGoal(goal);     
    }

    bool getResult() override {
        anytree_msgs::moveArmResultConstPtr result;
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