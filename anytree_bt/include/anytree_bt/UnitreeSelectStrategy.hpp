#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <anytree_msgs/selectStrategy.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <sstream>

using namespace BT;


//===========================================================================================================================================//

class UnitreeSelectStrategy : public BT::SyncActionNode {
  typedef ros::ServiceClient Client;

public:
  UnitreeSelectStrategy(const std::string &name,
                                const BT::NodeConfig &config,
                                ros::NodeHandle nh)
      : BT::SyncActionNode(name, config) {
    client = nh.serviceClient<anytree_msgs::selectStrategy>(
        "selectStrategy");
  }

  // You must override the virtual function tick()
  NodeStatus tick() override {
    auto device_type = getInput<std::string>("device_type");
    if (!device_type) {
      throw BT::RuntimeError("missing required input [device_type]");
    }
    auto device_id = getInput<std::string>("device_id");
    if (!device_id) {
      throw BT::RuntimeError("missing required input [device_id]");
    }
    auto input_strategy = getInput<int>("input_strategy");

    anytree_msgs::selectStrategy srv;
    srv.request.device_type = device_type.value();
    srv.request.device_id = device_id.value();
    if (input_strategy) {
      srv.request.input_strategy = input_strategy.value();
    }

    if (client.call(srv)) {
      std::cout << "Selected Strategy: "
                << static_cast<int>(srv.response.selected_strategy) << std::endl;
      setOutput("selected_strategy", static_cast<int>(srv.response.selected_strategy));
      UnitreeJointEfforts strategy_effort_limit;
      strategy_effort_limit.j1_effort = srv.response.strategy_effort_limit[0];
      strategy_effort_limit.j2_effort = srv.response.strategy_effort_limit[1];
      strategy_effort_limit.j3_effort = srv.response.strategy_effort_limit[2];
      strategy_effort_limit.j4_effort = srv.response.strategy_effort_limit[3];
      strategy_effort_limit.j5_effort = srv.response.strategy_effort_limit[4];
      strategy_effort_limit.j6_effort = srv.response.strategy_effort_limit[5];
      std::cout << "Strategy Effort Limit: " << strategy_effort_limit << std::endl;
      setOutput("strategy_effort_limit", strategy_effort_limit);
      setOutput("min_angle", static_cast<double>(srv.response.min_angle));
      setOutput("max_angle", static_cast<double>(srv.response.max_angle));
      setOutput("rot_sym_angle", static_cast<double>(srv.response.rot_sym_angle));
      UnitreeJointEfforts device_effort_limit;
      device_effort_limit.j1_effort = srv.response.device_effort_limit[0];
      device_effort_limit.j2_effort = srv.response.device_effort_limit[1];
      device_effort_limit.j3_effort = srv.response.device_effort_limit[2];
      device_effort_limit.j4_effort = srv.response.device_effort_limit[3];
      device_effort_limit.j5_effort = srv.response.device_effort_limit[4];
      device_effort_limit.j6_effort = srv.response.device_effort_limit[5];
      std::cout << "Device Effort Limit: " << device_effort_limit << std::endl;
      setOutput("device_effort_limit", device_effort_limit);
      return NodeStatus::SUCCESS;
    } else {
      std::cout
          << "Did not recieve decision from selectStrategy Service!"
          << std::endl;
      return NodeStatus::FAILURE;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("device_type"),
            BT::InputPort<std::string>("device_id"),
            BT::InputPort<int>("input_strategy"),
            BT::OutputPort<int>("selected_strategy"),
            BT::OutputPort<UnitreeJointEfforts>("strategy_effort_limit"),
            BT::OutputPort<double>("min_angle"),
            BT::OutputPort<double>("max_angle"),
            BT::OutputPort<double>("rot_sym_angle"),
            BT::OutputPort<UnitreeJointEfforts>("device_effort_limit")};
  }

private:
  Client client;
};