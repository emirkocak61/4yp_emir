#include <anytree_control/UnitreeRosSim.hpp>

int main(int argc,char** argv) {
    ros::init(argc,argv,"unitree");
    UnitreeRosSim unitree_interface;
    unitree_interface.startPublishing();
    ros::spin();
    unitree_interface.startPublishing();
    return 0;
}
