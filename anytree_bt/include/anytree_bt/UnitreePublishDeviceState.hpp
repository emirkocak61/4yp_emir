#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

//===========================================================================================================================================//

/**
 * @brief Synchronous ActionNode to publish the current device state
 *
 */
class UnitreePublishDeviceState : public BT::SyncActionNode {
public:
  UnitreePublishDeviceState(const std::string &name,
                           const BT::NodeConfig &config, ros::NodeHandle nh)
      : BT::SyncActionNode(name, config) {
    nh_ = nh;
    pub = nh_.advertise<std_msgs::Float64>("device_state",10);

  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override {
    auto device_state = getInput<double>("device_state");
    if (!device_state) {
      throw BT::RuntimeError("missing required input [device_state]");
    }

    std_msgs::Float64 device_state_msg;
    device_state_msg.data = device_state.value();

    pub.publish(device_state_msg);

    return BT::NodeStatus::SUCCESS;
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("device_state")};
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub;
};
