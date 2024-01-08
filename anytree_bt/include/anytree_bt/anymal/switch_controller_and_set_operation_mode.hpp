#pragma once

#include <anymal_motion_control_msgs/ExecuteMotionAction.h>
#include <anytree_bt/bt_action_client_node.hpp>
#include <behaviortree_cpp/behavior_tree.h>

/**
 * @brief Asynchronous ActionNode for switching operational modes 
 * of a controller
*/

class SwitchControllerAndSetOperationModeAnymal : public ActionClientNode {
    typedef actionlib::SimpleActionClient<
    anymal_motion_control_msgs::ExecuteMotionAction> Client;

public:
    SwitchControllerAndSetOperationModeAnymal(const std::string& name,
                                              const BT::NodeConfig& config)
        : ActionClientNode(name,config) {
            client_ptr = std::unique_ptr<Client>(
                new Client("/motion_control_manager/force_motion_execution", true)
            );
        }
    ~SwitchControllerAndSetOperationModeAnymal() override {}

    static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("motion_controller"),
            BT::InputPort<int>("reference_type"),
            BT::InputPort<std::string>("operation_mode"),
            BT::InputPort<std::string>("motion_parameters")};
}

    void sendStartRequest() override {
        client_ptr->waitForServer();
        anymal_motion_control_msgs::ExecuteMotionGoal goal;

        auto motion_controller = getInput<std::string>("motion_controller");
        if (!motion_controller) {
            throw BT::RuntimeError("missing required input [motion controller]");
        }
        auto reference_type = getInput<int>("reference_type");
        if (!reference_type) {
            throw BT::RuntimeError("missing required input [reference_type]");
        } else {
            if (reference_type.value() < 0 || reference_type.value() > 3) {
                throw BT::RuntimeError("input [reference_type] needs to be 0 (N/A), 1 "
                                    "(pose), 2 (twist), or 3 (action)");
                }
        }
        auto operation_mode = getInput<std::string>("operation_mode");
        if (!operation_mode) {
            throw BT::RuntimeError("missing required input [operation_mode]");
        }
        auto motion_parameters = getInput<std::string>("motion_parameters"); // optional

        goal.motion.motion_controller = motion_controller.value();
        goal.motion.reference_type.data = reference_type.value();
        goal.motion.operation_mode = operation_mode.value();
        if (motion_parameters) {
            goal.motion_parameters = motion_parameters.value();
        }

        std::cout << "Requesting switch to controller '"
              << motion_controller.value() << "' (reference type "
              << reference_type.value() << ", operation mode '"
              << operation_mode.value() << "')" << std::endl;

        client_ptr->sendGoal(goal);
    }

    bool getResult() override {
        anymal_motion_control_msgs::ExecuteMotionResultConstPtr result =
        client_ptr->getResult();
        if (result->execution_result.data !=
        anymal_motion_control_msgs::MotionExecutionResult::RESULT_EXECUTED) {
        std::cerr
          << "Failed to switch controller and operation mode (status code "
          << result->execution_result.data << ")" << std::endl;
        }
        return (result->execution_result.data ==
            anymal_motion_control_msgs::MotionExecutionResult::RESULT_EXECUTED);
    }

    State getState() override {return client_ptr->getState();}

    void sendAbortSignal() override {client_ptr->cancelGoal();}

    
private:
    std::unique_ptr<Client> client_ptr;

};