#ifndef EXAMPLE_NODES_HPP
#define EXAMPLE_NODES_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include <iostream>
#include <string>

namespace ExampleNodes {

class ApproachObject : public BT::SyncActionNode {
public:
    ApproachObject(const std::string& name);
    BT::NodeStatus tick() override;
};

BT::NodeStatus CheckBattery();

class GripperInterface {
public:
    GripperInterface();
    BT::NodeStatus open();
    BT::NodeStatus close();
private:
    bool _open;
};

class SaySomething : public BT::SyncActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }
};
} // namespace example nodes

#endif //EXAMPLE_NODES_HPP