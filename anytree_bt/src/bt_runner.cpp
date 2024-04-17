#include <behaviortree_cpp/bt_factory.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <anytree_bt/UnitreeGripperCommand.hpp>
#include <anytree_bt/UnitreeMoveArm.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_tree");

  if (argc != 3) {
    std::cerr << "This node assumes two input arguments: the XML file name and the "
                 "name of the tree to run"
              << std::endl;
    return EXIT_FAILURE;
  }

  constexpr bool verbose{false};

  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;

  // To use compiled shared libraries
  factory.registerFromROSPlugins();

  factory.registerNodeType<UnitreeGripperCommand>("UnitreeGripperCommand");
  factory.registerNodeType<UnitreeMoveArm>("UnitreeMoveArm");

  std::cout << "Loading Behavior Tree..." << std::endl;
  factory.registerBehaviorTreeFromFile(ros::package::getPath("anytree_bt") + "/resources/trees/" + argv[1] + ".xml");

  std::cout << "Building Behavior Tree..." << std::endl;
  auto tree = factory.createTree(argv[2]);
  if (verbose)
    printTreeRecursively(tree.rootNode());

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::NodeStatus status;

  status = tree.tickWhileRunning();
  std::cout << "Tree completed, final status: " << BT::toStr(status)
            << std::endl;

  // // Keep ticking until behavior returns SUCCESS
  // while (status != BT::NodeStatus::SUCCESS) {
  //   if (verbose)
  //     std::cout << "--- ticking\n";
  //   status = tree.tickOnce();
  //   if (verbose)
  //     std::cout << "--- status: " << BT::toStr(status) << "\n\n";

  //   // if still running, add some wait time
  //   if (status == BT::NodeStatus::RUNNING) {
  //     tree.sleep(std::chrono::milliseconds(100));
  //   }
  // }

  return status == BT::NodeStatus::SUCCESS ? EXIT_SUCCESS : EXIT_FAILURE;
}
