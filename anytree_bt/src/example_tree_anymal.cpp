#include <behaviortree_cpp/bt_factory.h>
#include <anytree_bt/anymal/motion_transitioner.hpp>
#include <anytree_bt/anymal/navigate_to_goal.hpp>
#include <anytree_bt/ExampleNodes.hpp>
#include "ros/ros.h"
#include "ros/package.h"

using namespace ExampleNodes;

int main(int argc,char** argv) {
    ros::init(argc,argv, "example_bt_node");

    std::string package_path = ros::package::getPath("anytree_bt");
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<MotionTransitionerAnymal>("GoToMotionState");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<NavigateToGoalAnymal>("NavigateToGoal");

    std::string tree_path = package_path + "/resources/trees/anymal/navigate_to_goal.xml";
    BT::Tree tree = factory.createTreeFromFile(tree_path);

    tree.tickWhileRunning();
}
