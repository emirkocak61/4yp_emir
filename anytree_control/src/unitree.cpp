#include <anytree_control/UnitreeRos.hpp>

int main(int argc,char** argv) {
    ros::init(argc,argv,"unitree");
    ros::NodeHandle nh;
    bool isSim;
    if (nh.getParam("isSim", isSim)) {
        UnitreeRos arm_interface(isSim);
        arm_interface.startPublishing();
        ros::spin();
        arm_interface.stopPublishing();
        return 0;
    } else {
        ROS_WARN("Please set the 'isSim' parameter before running this file");
        return -1;
    }   
}