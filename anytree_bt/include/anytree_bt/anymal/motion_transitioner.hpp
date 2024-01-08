#pragma once

#include <anytree_bt/bt_action_client_node.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <motion_transitioner_msgs/GoToMotionStateAction.h>

/**
 * @brief Asynchronous ActionNode for transitioning between smart motion states
*/
class MotionTransitionerAnymal : public ActionClientNode {
    
    typedef actionlib::SimpleActionClient<
        motion_transitioner_msgs::GoToMotionStateAction> Client;

public:
    MotionTransitionerAnymal(const std::string& name,const BT::NodeConfig &config) :
        ActionClientNode(name, config){

            client_ptr = std::unique_ptr<Client>(
                new Client("/motion_control_manager/go_to_motion_state", true)
            );
        }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("goal_motion_state"),
                BT::InputPort<std::string>("motion_parameters")};
    }

    void sendStartRequest() override {
        client_ptr->waitForServer();
        motion_transitioner_msgs::GoToMotionStateGoal goal;
        
        auto goal_motion_state = getInput<std::string>("goal_motion_state");
        if (!goal_motion_state) {
            throw BT::RuntimeError("missing required input [goal_motion_state]");
        }

        auto motion_parameters = getInput<std::string>("motion_parameters"); //optional
        goal.goal_motion_state = goal_motion_state.value();
        if (motion_parameters) {
            goal.motion_parameters = motion_parameters.value();
        }
        goal.abort_on_param_failure = true;
        std::cout << "Requesting transition to motion state '"
                  << goal.goal_motion_state << "'(parameters: '"
                  << goal.motion_parameters << "')" << std::endl;
        client_ptr->sendGoal(goal);
    }

    bool getResult() override {
        motion_transitioner_msgs::GoToMotionStateResultConstPtr result;
        result = client_ptr->getResult();
        return result->result;
    }

    State getState() override {return client_ptr->getState();}

    void sendAbortSignal() override {client_ptr->cancelGoal();}
private:
    std::unique_ptr<Client> client_ptr;
};
