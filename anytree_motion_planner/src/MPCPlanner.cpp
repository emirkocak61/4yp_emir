#include <anytree_motion_planner/MPCMotionPlanner.hpp>

int main(int argc,char** argv) {
    ros::init(argc,argv,"MPCPlanner_node");
    ros::NodeHandle nh;
    //Construct the motion planner
    MPCMotionPlanner planner;
    //Set ros spinners
    ros::AsyncSpinner spinner(3);
    spinner.start();
    //Wait for shutdown
    ros::waitForShutdown();
    return 0;
}