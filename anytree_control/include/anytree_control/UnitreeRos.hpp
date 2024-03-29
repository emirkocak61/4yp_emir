#pragma once
/*
    The class that interfaces the unitree arm with ROS. It includes the feedforward control function and publisher that
    continuously publishes state information into a topic. The velocity state is low-pass filtered. 
*/
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <anytree_control/LowPassFilter.hpp>
#include <mutex>

class UnitreeRos {
public:
    UnitreeRos() : 
    arm(true), 
    dt(0.05),
    velocity_filter(0.167,Eigen::VectorXd::Zero(arm_dof)), isPublishing(false) {
        SetupArm();
        reset_arm_sub = nh_.subscribe("/z1_gazebo/reset_arm_pose",10,&UnitreeRos::ResetArmPoseCb,this);
        motion_plan_sub = nh_.subscribe("/motion_plan", 10, &UnitreeRos::MotionPlanCb, this);
        gripper_sub = nh_.subscribe("/z1_gazebo/gripper_command",10,&UnitreeRos::GripperMotionCb,this);
        state_publisher = nh_.advertise<sensor_msgs::JointState>("/z1_gazebo/joint_states_filtered",10);   //z1_gazebo/joint_states/filtered  
    }
    ~UnitreeRos() {
        // Stop the publishing thread if it's running
        if (publisher_thread.joinable()) {
            isPublishing = false;
            publisher_thread.join();
        }
    }

    void GripperMotionCb(const std_msgs::Bool& msg) {
        double targetQ;
        if (msg.data == true) {
            //Check if the gripper is closed
            if (isGripperOpen_ == true) {
                ROS_WARN("Gripper is already open");
                return;
            }
            else {
                //Open the gripper
                isGripperOpen_ = true;
                targetQ = -0.50; 
                sendGripperCommand(targetQ);
                return; 
            }
        }
        else {
            //Check if the gripper is open
            if (isGripperOpen_ == false) {
                ROS_WARN("Gripper is already closed");
                return;
            }
            else {
                isGripperOpen_ = false;
                targetQ = -0.001;
                sendGripperCommand(targetQ);
                return;
            }
        }
    }

    void sendGripperCommand(const double& targetQ) {
        arm.sendRecvThread->shutdown();
        double initQ = arm.lowstate->getGripperQ();
        double duration = 500;
        UNITREE_ARM::Timer timer(0.01);
        for (int i(0); i < duration; i++) {
            //Set the gripper commands by linear interpolation
            arm.gripperQ = initQ * (1-i/duration) + targetQ * (i/duration);
            arm.gripperW = (targetQ - initQ) / (duration * 0.01);

            //Use the latest arm commands to keep the arm steady
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            //Set the gripper commands, assuming 0 torque
            arm.setGripperCmd(arm.gripperQ,arm.gripperW);
            arm.sendRecv();
            timer.sleep();
        }
        arm.sendRecvThread->start();
    }

    void MotionPlanCb(const trajectory_msgs::JointTrajectory& msg) {
        motion_plan = msg.points[0];
        // Use Eigen::Map to directly map the positions to an Eigen::VectorXd
        Eigen::Map<const Eigen::VectorXd> positions_map(motion_plan.positions.data()+6, arm_dof);
        Eigen::Map<const Eigen::VectorXd> velocities_map(motion_plan.velocities.data()+6, arm_dof);
        double duration = 10;
        arm.q = positions_map;
        arm.qd = velocities_map;
        sendArmCommand(positions_map, duration);
    }
    //Sends the arm to home position
    void ResetArmPoseCb(const std_msgs::Bool &message) {
        if(message.data == true) {
            Vec6 homeQ;
            double duration = 500;
            homeQ << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0;
            sendArmCommand(homeQ,duration);    
        }
    }
    
    void sendArmCommand(const Eigen::VectorXd& targetQ,const double duration) {
        arm.sendRecvThread->shutdown();
        Vec6 initQ = arm.lowstate->getQ();
        UNITREE_ARM::Timer timer(0.002);
        //Timer timer(control_frequency);
        for(int i(0); i<duration; i++){
            arm.q =  initQ * (1-i/duration) + targetQ * (i/duration);
            arm.qd = (targetQ - initQ) / (duration * 0.01);
            arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
            //gripperQ = -0.001;
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            //setGripperCmd(gripperQ, gripperW, gripperTau);
            arm.sendRecv();
            timer.sleep();
        }
        arm.sendRecvThread->start();
    }

    void SetupArm() {
        //Start communications with arm controller
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        std::vector<double> KP = {100,150,150,100,75,50};
        std::vector<double> KD = {500,500,500,500,500,500};

        //Set control gains
        arm.lowcmd->setControlGain(KP,KD);
    }

    void startPublishing() {
        if (!isPublishing) {
            isPublishing = true;
            publisher_thread = std::thread(&UnitreeRos::publishLoop, this);
        }
    }

    void stopPublishing() {
        if (isPublishing) {
            isPublishing = false;
            if (publisher_thread.joinable()) {
                publisher_thread.join();
            }
        }
    }

private:
    void publishLoop() {
        ros::Rate rate(50); //50Hz
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.position.resize(6);
        joint_state_msg.velocity.resize(6);
        while (ros::ok() && isPublishing) {
            //std::unique_lock<std::mutex> lock(lowstate_mutex);
            Vec6 arm_position = arm.lowstate->getQ();
            Vec6 arm_velocity = arm.lowstate->getQd();
            //lock.unlock();
            Vec6 v_filtered = velocity_filter.filter(arm_velocity);
            // Directly assign values using Eigen::Map
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.position.data(), arm_dof) = arm_position;
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.velocity.data(), arm_dof) = v_filtered;

            //Publish message
            state_publisher.publish(joint_state_msg);
            rate.sleep();
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber motion_plan_sub;
    ros::Subscriber gripper_sub;
    ros::Subscriber reset_arm_sub;
    ros::Publisher state_publisher;
    int arm_dof = 6;
    double dt;
    bool isGripperOpen_ = false;

    LowPassFilter<Eigen::VectorXd> velocity_filter; //LowPassFilter for velocity readings
    UNITREE_ARM::unitreeArm arm;
    trajectory_msgs::JointTrajectoryPoint motion_plan;

    std::thread publisher_thread;
    bool isPublishing;
    std::mutex lowstate_mutex;

};