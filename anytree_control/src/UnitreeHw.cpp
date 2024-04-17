#include <anytree_control/UnitreeRosHW.hpp>

int main(int argc,char** argv) {
    ros::init(argc,argv,"unitree");
    UnitreeRosHW unitree_interface;
    unitree_interface.startPublishing();
    ros::spin();
    unitree_interface.startPublishing();
    return 0;
}