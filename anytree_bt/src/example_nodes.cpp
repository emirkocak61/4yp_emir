#include "behaviortree_cpp/behavior_tree.h"
#include "anytree_bt/ExampleNodes.hpp"

namespace ExampleNodes {

ApproachObject::ApproachObject(const std::string& name) :
    BT::SyncActionNode(name, {}) 
{}

BT::NodeStatus ApproachObject::tick() {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckBattery() {
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

GripperInterface::GripperInterface() : _open(true) {}

BT::NodeStatus GripperInterface::open() {
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close() {
    _open = false;
    std::cout << "GripperInterface::close" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
} //Namespace ExampleNodes
