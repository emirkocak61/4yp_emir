#include <anytree_control/RobotInterface.hpp>

int main(int argc,char**argv) {
    ros::init(argc, argv, "robot_interface_node");
    ros::NodeHandle nh;
    
    RobotInterface robotInterface(0.05);
    
    // Start the spinner with 2 threads
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}
