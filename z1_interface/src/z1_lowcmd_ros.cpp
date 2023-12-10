#include "ros/ros.h"
#include "unitree_arm_sdk/control/unitreeArm.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <vector>
#include <iostream>
#include <stdexcept>

using namespace UNITREE_ARM;

//Class for interfacing Unitree Sdk with ROS
class UnitreeInterface : public unitreeArm {
public:
    ros::NodeHandle nh_; //Node handle 
    ros::Subscriber sub;
    std::string ctrl_mode; //Subscriber to receive pose goals
    //std::vector<double> KP, KW; //Variables to store control gains

    UnitreeInterface(std::string control_mode) : unitreeArm(true) {
        
        ROS_INFO_STREAM("Z1 Interface Node is running, waiting for motion plans");
        this->ctrl_mode = control_mode; //Defines which control mode to use
        sub = nh_.subscribe("/motion_plan", 10, &UnitreeInterface::z1command, this); //set up the subscriber
        }
    
    ~UnitreeInterface() {}
    //A function to set the control mode of the arm, not sure if the error handling is necessary
    //I only added for the sake of experience and learning
    //TO-DO:  add other control modes

    
    void set_ctrl_mode(std::string ctrl_mode) {
        try {
            check_ctrl_mode(ctrl_mode);
        } catch (const std::runtime_error &e)  {
            std::cerr << "Caught an exception: " << e.what() << std::endl;
            //Set the control mode to passive
            setFsm(ArmFSMState::PASSIVE);
            sendRecv();
        }
        setFsm(ArmFSMState::PASSIVE);
        sendRecv();
        setFsm(ArmFSMState::LOWCMD);
        sendRecv();
    }

    bool check_ctrl_mode(std::string mode) {
        if (mode == "lowcmd") {
            return true;
        }
        else {
            throw std::runtime_error("No such control mode, setting to passive");
        }

    }

    ~UnitreeInterface() {}

    void z1command(const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {
        ROS_INFO_STREAM("Received Motion Plan, executing...");
        std::cout << std::fixed << std::setprecision(3);
        sendRecvThread->start();
        labelRun("startFlat");
        setFsm(ArmFSMState::PASSIVE);
        setFsm(ArmFSMState::LOWCMD);
        std::vector<double> KP, KW;
        KP = _ctrlComp->lowcmd->kp;
        KW = _ctrlComp->lowcmd->kd;
        _ctrlComp->lowcmd->setControlGain(KP, KW);
        sendRecvThread->shutdown();
        Vec6 initQ = lowstate->getQ(); //Get current arm position
        double duration = 1000;
        std::vector<double> Q = msg->positions;
        Vec6 targetQ; 
        targetQ << Q[0], Q[1], Q[2], Q[3], Q[4], Q[5];
        Timer timer(_ctrlComp->dt);
        for(int i(0); i<duration; i++){
            q = initQ * (1-i/duration) + targetQ * (i/duration);
            qd = (targetQ - initQ) / (duration * _ctrlComp->dt);
            tau = _ctrlComp->armModel->inverseDynamics(q, qd, Vec6::Zero(), Vec6::Zero());
            gripperQ = -(i/duration);
            
            setArmCmd(q, qd, tau);
            setGripperCmd(gripperQ, gripperW, gripperTau);
            sendRecv();
            timer.sleep();
        }
        sendRecvThread->start();
        setFsm(ArmFSMState::JOINTCTRL);
        labelRun("startFlat");
        setFsm(ArmFSMState::PASSIVE);
        sendRecvThread->shutdown();
    }

}; 

int main(int argc,char** argv){
    ros::init(argc,argv, "z1_interface_node");
    UnitreeInterface arm("lowcmd");
    ros::spin();
}


