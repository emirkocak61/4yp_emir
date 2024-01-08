#pragma once

#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

/**
 * @brief Abstract class to connect a ROS action server as an asynchronous 
 * Action node. Specific implementations can derive from this class
*/

class ActionClientNode : public BT::StatefulActionNode {
public:
    typedef actionlib::SimpleClientGoalState State;

    ActionClientNode(const std::string& name, const BT::NodeConfig &config) :
        BT::StatefulActionNode(name, config) {}
    ~ActionClientNode() override {}

    BT::NodeStatus onStart() override {
        //Send a request to the server
        sendStartRequest();

        State state = getState();
        //Check if the request was rejected by the server
        if (state == State::StateEnum::REJECTED) {
            return BT::NodeStatus::FAILURE;
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    //Method invoked by an action in the running state
    BT::NodeStatus onRunning() override {
        State state = getState();

        if (state == State::StateEnum::SUCCEEDED){
            //retrieve the result
            bool result = getResult();
            if (result) {
                return BT::NodeStatus::SUCCESS;
            } else { return BT::NodeStatus::FAILURE;}
        } else if ((state == State::StateEnum::ABORTED) ||
               (state == State::StateEnum::RECALLED) ||
               (state == State::StateEnum::PREEMPTED) ||
               (state == State::StateEnum::LOST)) {
            // fail if the action was aborted by some other client
            // or by the server itself
            std::cerr << "[ActionClient] Failure due to: ";
            if (state == State::StateEnum::ABORTED)
                std::cerr << "ABORTED" << std::endl;
            if (state == State::StateEnum::RECALLED)
                std::cerr << "RECALLED" << std::endl;
            if (state == State::StateEnum::PREEMPTED)
                std::cerr << "PREEMPTED" << std::endl;
            if (state == State::StateEnum::LOST)
                std::cerr << "LOST" << std::endl;

            return BT::NodeStatus::FAILURE;
        } else {
        // probably (state == ACTIVE)
        return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        //Notify the server that the operation has been aborted
        std::cerr << "Halt Triggered!" << std::endl;
        sendAbortSignal();
    }

    //Pure virtual functions, must be implemented in the derived classes
    virtual void sendStartRequest() = 0;
    virtual void sendAbortSignal() = 0;
    virtual bool getResult() = 0;
    virtual State getState() = 0;
};