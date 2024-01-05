#include "behaviortree_cpp/bt_factory.h"
#include "ros/package.h"

//File that contains custom node definitions
#include "anytree_bt/ExampleNodes.hpp"

using namespace ExampleNodes;

int main() {
    std::string package_path = ros::package::getPath("anytree_bt");
    //Use behavior tree factory to register custom nodes
    BT::BehaviorTreeFactory factory;

    //Create a node through inheritence
    factory.registerNodeType<ApproachObject>("ApproachObject");

    //Register a SimpleActionNode using function pointer
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    //Create a simple action node using a method from a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper",std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper",std::bind(&GripperInterface::close,&gripper));

    //Load tree from xml file
    std::string tree_path = package_path + "/resources/trees/example_tree.xml";
    BT::Tree tree = factory.createTreeFromFile(tree_path);

    tree.tickWhileRunning();

    return 0;

}


