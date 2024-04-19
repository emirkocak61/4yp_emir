#include <behaviortree_cpp/bt_factory.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <anytree_bt/UnitreeGripperCommand.hpp>
#include <anytree_bt/UnitreeMoveArm.hpp>
#include <anytree_bt/UnitreeMonitorEffort.hpp>

//===========================================================================================================================================//

int main(int argc, char **argv) {
  ros::init(argc, argv, "bt_standalone_arm_test");

  ros::NodeHandle nh;

  ros::Rate rate(100);

  BT::BehaviorTreeFactory factory;

  // To use compiled shared libraries
  factory.registerFromROSPlugins();

  factory.registerNodeType<UnitreeGripperCommand>("UnitreeGripperCommand");
  factory.registerNodeType<UnitreeMoveArm>("UnitreeMoveArm");
  factory.registerNodeType<UnitreeMonitorEffort>("UnitreeMonitorEffort", nh);

  std::cout << "Loading Behavior Tree..." << std::endl;

  factory.registerBehaviorTreeFromFile(ros::package::getPath("anytree_bt") +
                                       "/resources/trees/bt_standalone_arm_test.xml");

  // // Find all the XML files in a folder and register all of them.
  // // We will use std::filesystem::directory_iterator
  // std::string search_directory = "./";

  // using std::filesystem::directory_iterator;
  // for (auto const& entry : directory_iterator(search_directory))
  // {
  //   if( entry.path().extension() == ".xml")
  //   {
  //     factory.registerBehaviorTreeFromFile(entry.path().string());
  //   }
  // }

  std::cout << "Building Behavior Tree..." << std::endl;

  // TODO: Use parameter for node to call tree
  auto tree = factory.createTree("MainTree");
  // printTreeRecursively(main_tree.rootNode());

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::NodeStatus status;
  // Keep ticking until behavior returns SUCCESS or FAILURE
  while ((status != BT::NodeStatus::SUCCESS) &&
         (status != BT::NodeStatus::FAILURE)) {
    status = tree.tickOnce();
    rate.sleep();
  }

  return status == BT::NodeStatus::SUCCESS ? EXIT_SUCCESS : EXIT_FAILURE;
}



