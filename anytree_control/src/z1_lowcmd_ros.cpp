//ROS Interface to control unitree arm in lowcmd mode

#include "ros/ros.h"
#include <std_msgs/Bool.h>
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
    ros::Subscriber sub1; //Subscriber to receive pose goals
    ros::Subscriber sub2; //Subscriber to receive gripper commands
    ros::Subscriber sub3; //Subscriber to send arm to rest pose
    Timer timer;

    std::vector<double> KP, KW; //Variables to store control gains
    double  dt = 0.02;
    double gripperQ; //Gripper position
    

    UnitreeInterface(std::vector<double> KP = {20,30,30,20,15,10}, 
    std::vector<double> KW ={2000,2000,2000,2000,2000,2000}) 
        //Parent class constructor
        : unitreeArm(true), 
        //Sets a timer for 500Hz
        timer(_ctrlComp->dt) {
        
        ROS_INFO_STREAM("Z1 Control Interface Node is running, waiting for motion plans");
        std::cout << std::fixed << std::setprecision(3);
        //Sets the control mode to lowcmd
        this->KP = KP;
        this->KW = KW;
        gripperQ = -0.001;
        sub1 = nh_.subscribe("/motion_plan", 10, &UnitreeInterface::z1command, this); //set up the subscriber
        sub2 = nh_.subscribe("/gripper_command",10,&UnitreeInterface::gripperCb,this);
        sub3 = nh_.subscribe("/reset_arm_pose",10,&UnitreeInterface::ResetArmPose,this);
        sendRecvThread->start();
        setControlMode(); //Sets control mode to lowcmd
        
        }
    
    ~UnitreeInterface() {}

    void GripperCb(const std_msgs::Bool &msg) { //1 for open 0 for close
        sendRecvThread->shutdown();
        std::cout << "Received gripper goal" << std::endl;
        double duration = 3000;
        if(msg.data == true) {
            gripperQ = -0.50;
            std::cout << "Opening gripper, gripperQ: " << gripperQ << std::endl;}
        else {
            gripperQ = -0.001;
            std::cout << "Closing gripper, gripperQ: " << gripperQ << std::endl;
        }
        //Gets current gripper state
        Timer timer2(_ctrlComp->dt);
        double Q = lowstate->getGripperQ();
        for(int i(0); i < duration; i++){
                double q_gripper = Q * (1-i/duration) + gripperQ * (i/duration);
                setGripperCmd(q_gripper,0.05,0); //Set gripper velocity to 0.05 to avoid very fast motions
                sendRecv();
                timer2.sleep();
            }
        sendRecvThread->start();
    } 

    void gripperCb(const std_msgs::Bool &msg) {
        if (msg.data == true) {gripperQ = -0.50;}
        if (msg.data == false) {gripperQ = -0.05;}
        std::cout << "New gripper command" << gripperQ << std::endl;
    }
    
    //The callback function for the /motion_plan topic
    void z1command(const trajectory_msgs::JointTrajectoryConstPtr &msg) {
        //ROS_INFO_STREAM("Received Motion Plan, executing...");
        //set_ctrl_mode();
        //double duration = 1000;
        trajectory_msgs::JointTrajectoryPoint trajectory_point = msg->points[0];
        std::vector<double> Q = trajectory_point.positions; //This is a vector with 12 elements (6 for base + 6 for arm)
        Vec6 motion_goal; 
        
        //Extract the joint configurations for the arm
        motion_goal << Q[6], Q[7], Q[8], Q[9], Q[10], Q[11];
        //Write the received joint configurations for checking  
        sendArmGoal(motion_goal);
        
    }
    
    //Function that implements the torque calculation and communication with arm
    void sendArmGoal(const Vec6 targetQ) {
        sendRecvThread->shutdown();
        Vec6 initQ = lowstate->getQ();
        double duration = 20; 
        Timer timer(_ctrlComp->dt); //This is the default control frequency of 500Hz
        //Timer timer(control_frequency);
        for(int i(0); i<duration; i++){
            q = initQ * (1-i/duration) + targetQ * (i/duration);
            qd = (targetQ - initQ) / (duration * _ctrlComp->dt);
            tau = _ctrlComp->armModel->inverseDynamics(q, qd, Vec6::Zero(), Vec6::Zero());
            //gripperQ = -0.001;
            setArmCmd(q, qd, tau);
            setGripperCmd(gripperQ, gripperW, gripperTau);
            sendRecv();
            timer.sleep();
        }
        sendRecvThread->start();
        
    }

    //Communicates with the arm to set control gains
    void setCntrlGain() {
        ROS_INFO_STREAM("Setting control gain");
        //Set the control gain
        lowcmd->setControlGain(KP,KW);;
    }
    
    //Sends the arm to home position
    void ResetArmPose(const std_msgs::Bool &message) {
        if(message.data == true) {
            Vec6 homeQ;
            double duration = 1000;
            homeQ << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0; 
            sendRecvThread->shutdown();
            Vec6 initQ = lowstate->getQ();
            //Set the control frequncy same as 50Hz
            Timer timer3(_ctrlComp->dt); //This is the default control frequency of 500Hz
            //Timer timer(control_frequency);
            for(int i(0); i<duration; i++){
                q = initQ * (1-i/duration) + homeQ * (i/duration);
                qd = (homeQ - initQ) / (duration * _ctrlComp->dt);
                tau = _ctrlComp->armModel->inverseDynamics(q, qd, Vec6::Zero(), Vec6::Zero());
                double gripper = - (i/(3*duration));
                
                setArmCmd(q, qd, tau);
                setGripperCmd(gripper, gripperW, gripperTau);
                sendRecv();
                timer3.sleep();
            }
            sendRecvThread->start();
        }
    }

    

    //Communicates with the arm to set control mode to lowcmd
    void setControlMode() {
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
    std::vector<double> gain = {100,150,150,100,75,50}; //Multiplied by a factor of 5
    //Instantiate the arm interface with the custom control gains
    UnitreeInterface arm(gain);

    //arm.setControlMode(); //Communicates with the arm to set control mode to lowcmd
    arm.lowcmd->setControlGain(arm.KP,arm.KW); //Communicates with the arm to set control gains
    ros::spin();
}

