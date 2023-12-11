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
    ros::Subscriber sub; //Subscriber to receive pose goals
    std::string ctrl_mode = "lowcmd";
    std::vector<double> KP, KW; //Variables to store control gains
    

    UnitreeInterface(std::string control_mode,std::vector<double> KP = {20,30,30,20,15,10}, 
    std::vector<double> KW ={2000,2000,2000,2000,2000,2000}) : unitreeArm(true) {
        
        ROS_INFO_STREAM("Z1 Interface Node is running, waiting for motion plans");
        std::cout << std::fixed << std::setprecision(3);
        this->ctrl_mode = control_mode; //Defines which control mode to use
        this->KP = KP;
        this->KW = KW;
        sub = nh_.subscribe("/motion_plan", 10, &UnitreeInterface::z1command, this); //set up the subscriber
        sendRecvThread->start();
        }
    
    ~UnitreeInterface() {}
    
    //The callback function for the /motion_plan topic
    void z1command(const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {
        ROS_INFO_STREAM("Received Motion Plan, executing...");
        set_ctrl_mode();
        double duration = 1000;
        std::vector<double> Q = msg->positions;
        Vec6 motion_goal; 
        motion_goal << Q[0], Q[1], Q[2], Q[3], Q[4], Q[5];
        run(motion_goal, duration);
        
        
    }
    
    //Function that implements the torque calculation and communication with arm
    void run(Vec6 targetQ, double duration) {
        sendRecvThread->shutdown();
        Vec6 initQ = lowstate->getQ();
        Timer timer(_ctrlComp->dt);
        for(int i(0); i<duration; i++){
            q = initQ * (1-i/duration) + targetQ * (i/duration);
            qd = (targetQ - initQ) / (duration * _ctrlComp->dt);
            tau = _ctrlComp->armModel->inverseDynamics(q, qd, Vec6::Zero(), Vec6::Zero());
            gripperQ = -0.001;
            
            setArmCmd(q, qd, tau);
            setGripperCmd(gripperQ, 0, 0);
            sendRecv();
            timer.sleep();
        }
        sendRecvThread->start();
    }

    void set_ctrl_gain() {
        ROS_INFO_STREAM("Setting control gain");
        //Set the control gain
        lowcmd->setControlGain(KP,KW);;
    }
    
    void go_to_start() {
        Vec6 home_pose;
        double duration = 1000;
        home_pose << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0; 
        run(home_pose, duration);
    }

    

    //A function to set the control mode of the arm to lowcmd
    void set_ctrl_mode() {
        // Check if the current state is LOWCMD
        if (_ctrlComp->recvState.state != ArmFSMState::LOWCMD) { 
            // Set to LOWCMD if it's not already
            ROS_INFO_STREAM("Setting control mode to lowcmd");
            setFsm(ArmFSMState::PASSIVE);
            setFsm(ArmFSMState::LOWCMD);
            return;  
        }
        else {
            ROS_INFO_STREAM("Control mode already in lowcmd");
            return;   
        }
    }

    

    

}; 

int main(int argc,char** argv){
    ros::init(argc,argv, "z1_interface_node");
    std::vector<double> gain = {40,60,60,40,30,20};
    UnitreeInterface arm("lowcmd",gain);
    arm.set_ctrl_mode();
    arm.set_ctrl_gain();
    arm.go_to_start();
    ros::spin();
}


