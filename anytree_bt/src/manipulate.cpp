#include "behaviortree_cpp/bt_factory.h"
#include "ros/package.h"

//File that contains custom node definitions
#include <anytree_bt/anymal/motion_transitioner.hpp>
#include <anytree_bt/ApproachTarget.hpp>
#include <anytree_bt/ExampleNodes.hpp>


int main(int argc,char** argv) {
    ros::init(argc,argv,"example_bt_node");
    
    std::string package_path = ros::package::getPath("anytree_bt");
    //Use behavior tree factory to register custom nodes
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<MotionTransitionerAnymal>("GoToMotionState");
    factory.registerNodeType<ExampleNodes::SaySomething>("SaySomething");
    factory.registerNodeType<ApproachTarget>("ApproachTarget");
    //factory.registerNodeType<GraspTarget>("GraspTarget");
    //factory.registerNodeType<ManipulateTarget>("ManipulateTarget");
    //factory.registerNodeType<ResetArmPose>("ResetArmPose");

    
    //Load tree from xml file
    std::string tree_path = package_path + "/resources/trees/manipulate.xml";
    BT::Tree tree = factory.createTreeFromFile(tree_path);

    tree.tickWhileRunning();

    return 0;

}


