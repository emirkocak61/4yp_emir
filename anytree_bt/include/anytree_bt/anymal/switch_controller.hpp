#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <control_unit_msgs/SwitchController.h>
#include <ros/ros.h>

/**
 * @brief Synchronous ActionNode to switch the ANYmal base controller
 *
 */
class SwitchControllerAnymal : public BT::SyncActionNode {
public:
  SwitchControllerAnymal(const std::string &name, const BT::NodeConfig &config,
                         ros::NodeHandle nh)
      : BT::SyncActionNode(name, config), nh_(nh) {
    switch_controller_service_client_ =
        nh_.serviceClient<control_unit_msgs::SwitchController>(
            "/motion_control_manager/switch_controller");
    switch_controller_service_client_
        .waitForExistence(); // NB: Blocking call (!)
  }
  ~SwitchControllerAnymal() override {}

  static BT::PortsList providedPorts() {
    // name of controller to switch to
    return {BT::InputPort<std::string>("controller")};
  }

  BT::NodeStatus tick() override {
    auto controller_to_switch_to = getInput<std::string>("controller");

    if (!controller_to_switch_to) {
      std::cerr << "Missing parameter [controller]!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    control_unit_msgs::SwitchControllerRequest req;
    req.name = controller_to_switch_to.value();

    control_unit_msgs::SwitchControllerResponse resp;

    switch_controller_service_client_.call(req, resp);

    if (resp.status > 0) {
      std::cout << "Switched ANYmal controller to: "
                << controller_to_switch_to.value() << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cerr << "FAILED when switching ANYmal controller, status: "
                << static_cast<int>(resp.status) << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient switch_controller_service_client_;
};