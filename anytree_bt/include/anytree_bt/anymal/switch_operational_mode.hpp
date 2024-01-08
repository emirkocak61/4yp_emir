#pragma once

#include <anytree_bt/bt_action_client_node.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <operational_mode_manager_msgs/SwitchOperationalModeAction.h>

/**
 * @brief Asynchronous ActionNode for switching the operational mode
 *
 */
class SwitchOperationalModeAnymal : public ActionClientNode {
  typedef actionlib::SimpleActionClient<
      operational_mode_manager_msgs::SwitchOperationalModeAction>
      Client;

public:
  SwitchOperationalModeAnymal(const std::string &name,
                              const BT::NodeConfig &config)
      : ActionClientNode(name, config) {
    client_ptr = std::unique_ptr<Client>(
        new Client("/operational_mode_manager/switch_operational_mode", true));
  }
  ~SwitchOperationalModeAnymal() override {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("target")};
  }

  void sendStartRequest() override {
    client_ptr->waitForServer();
    operational_mode_manager_msgs::SwitchOperationalModeGoal goal;

    auto target = getInput<std::string>("target");
    if (!target) {
      throw BT::RuntimeError("missing required input [target]");
    }
    goal.target.name = target.value();
    goal.requester_id = "bt";

    std::cout << "Requesting switch to operational mode '" << target.value()
              << "'" << std::endl;

    client_ptr->sendGoal(goal);
  }

  bool getResult() override {
    operational_mode_manager_msgs::SwitchOperationalModeResultConstPtr result =
        client_ptr->getResult();
    if (result->result.status_code !=
            operational_mode_manager_msgs::SwitchStatus::RESULT_OK &&
        result->result.status_code !=
            operational_mode_manager_msgs::SwitchStatus::
                RESULT_CURRENT_IS_TARGET) {
      std::cerr << "Failed to switch operational mode (status code "
                << result->result.status_code
                << "): " << result->result.status_message << std::endl;
    }
    return (result->result.status_code ==
                operational_mode_manager_msgs::SwitchStatus::RESULT_OK ||
            result->result.status_code ==
                operational_mode_manager_msgs::SwitchStatus::
                    RESULT_CURRENT_IS_TARGET);
  }

  State getState() override { return client_ptr->getState(); }

  void sendAbortSignal() override { client_ptr->cancelGoal(); }

private:
  std::unique_ptr<Client> client_ptr;
};