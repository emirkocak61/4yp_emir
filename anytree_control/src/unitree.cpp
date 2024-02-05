#include <anytree_control/UnitreeRos.hpp>

int main(int argc,char** argv) {
    ros::init(argc,argv,"unitree_interface_node");
    UnitreeRos arm_interface;
    arm_interface.startPublishing();
    ros::spin();
    arm_interface.stopPublishing();
    return 0;
}