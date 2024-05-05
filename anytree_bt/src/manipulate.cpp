#include "behaviortree_cpp/bt_factory.h"
#include "ros/package.h"

//File that contains custom node definitions
#include <anytree_bt/anymal/motion_transitioner.hpp>
#include <anytree_bt/ApproachTarget.hpp>
#include <anytree_bt/ExampleNodes.hpp>
#include <anytree_bt/GraspTarget.hpp>
#include <anytree_bt/ManipulateTarget.hpp>
#include <anytree_bt/UnitreeGripperCommand.hpp>
#include <anytree_bt/UnitreeMoveArm.hpp>


int main(int argc,char** argv) {
    ros::init(argc,argv,"example_bt_node");
    
    std::string package_path = ros::package::getPath("anytree_bt");
    //Use behavior tree factory to register custom nodes
    BT::BehaviorTreeFactory factory;

    // To use compiled shared libraries
    factory.registerFromROSPlugins();

    factory.registerNodeType<MotionTransitionerAnymal>("GoToMotionState");
    factory.registerNodeType<ApproachTarget>("ApproachTarget");
    factory.registerNodeType<GraspTarget>("GraspTarget");
    factory.registerNodeType<ManipulateTarget>("ManipulateTarget");
    factory.registerNodeType<UnitreeGripperCommand>("UnitreeGripperCommand");
    factory.registerNodeType<UnitreeMoveArm>("UnitreeMoveArm");

    
    //Load tree from xml file
    //std::string tree_path = package_path + "/resources/trees/manipulate.xml";
    std::string tree_path = package_path + "/resources/trees/main.xml";
    BT::Tree tree = factory.createTreeFromFile(tree_path);

    tree.tickWhileRunning();

    return 0;

}


