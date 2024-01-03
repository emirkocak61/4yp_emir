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
    double control_frequency = 0.02;
    

    UnitreeInterface(std::string control_mode,std::vector<double> KP = {20,30,30,20,15,10}, 
    std::vector<double> KW ={2000,2000,2000,2000,2000,2000}) : unitreeArm(true) {
        
        ROS_INFO_STREAM("Z1 Control Interface Node is running, waiting for motion plans");
        std::cout << std::fixed << std::setprecision(3);
        this->ctrl_mode = control_mode; //Defines which control mode to use
        this->KP = KP;
        this->KW = KW;
        sub = nh_.subscribe("/motion_plan", 10, &UnitreeInterface::z1command, this); //set up the subscriber
        sendRecvThread->start();
        }
    
    ~UnitreeInterface() {}
    
    //The callback function for the /motion_plan topic
    void z1command(const trajectory_msgs::JointTrajectoryConstPtr &msg) {
        //ROS_INFO_STREAM("Received Motion Plan, executing...");
        //set_ctrl_mode();
        double duration = 1000;
        trajectory_msgs::JointTrajectoryPoint trajectory_point = msg->points[0];
        std::vector<double> Q = trajectory_point.positions; //This is a vector with 12 elements (6 for base + 6 for arm)
        Vec6 motion_goal; 
        //Get the joint configurations for the arm
        motion_goal << Q[6], Q[7], Q[8], Q[9], Q[10], Q[11];
        run(motion_goal, duration);
        
        
    }
    
    //Function that implements the torque calculation and communication with arm
    void run(const Vec6 targetQ, double duration) {
        sendRecvThread->shutdown();
        Vec6 initQ = lowstate->getQ();
        //Set the control frequncy same as 50Hz
        Timer timer(_ctrlComp->dt); //This is the default control frequency of 500Hz
        //Timer timer(control_frequency);
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

    //Communicates with the arm to set control gains
    void set_ctrl_gain() {
        ROS_INFO_STREAM("Setting control gain");
        //Set the control gain
        lowcmd->setControlGain(KP,KW);;
    }
    
    //Sends the arm to home position
    void go_to_start() {
        Vec6 home_config;
        double duration = 1000;
        home_config << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0; 
        run(home_config, duration);
    }

    

    //Communicates with the arm to set control mode to lowcmd
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
    //Define the control gains
    std::vector<double> gain = {40,60,60,40,30,20};
    //Instantiate the arm interface with the custom control gains
    UnitreeInterface arm("lowcmd",gain);

    arm.set_ctrl_mode(); //Communicates with the arm to set control mode to lowcmd
    arm.set_ctrl_gain(); //Communicates with the arm to set control gains
    arm.go_to_start(); //Sends the arm to home position
    ros::spin();
}


