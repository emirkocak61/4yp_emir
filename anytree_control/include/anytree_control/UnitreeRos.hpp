#pragma once
/*
    The class that interfaces the unitree arm with ROS. It includes the feedforward control function and publisher that
    continuously publishes state information into a topic. The velocity state is low-pass filtered. 
    We need different pd gains and state subscribers for HW and Sim. Accordingly, the setup function has two implementations
    depending on hardware or sim. 
*/
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include "actionlib/server/simple_action_server.h"
#include <std_msgs/Bool.h>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <anytree_control/LowPassFilter.hpp>
#include <anytree_msgs/gripperCommandAction.h>
#include <anytree_msgs/gripperCommandResult.h>
#include <mutex>

class UnitreeRos {
public:
    UnitreeRos(bool isSim): 
    arm(true), 
    dt(0.002),
    gripper_as(nh_as,"gripperCommand_as",boost::bind(&UnitreeRos::GripperMotionCb,this,_1),false),
    velocity_filter(0.167,Eigen::VectorXd::Zero(arm_dof)), isPublishing(false), isSim(isSim) {
        if (isSim) {
            SetupArmSim();
        } else {
            SetupArmHW();
        }
        SetJointStateMsg();
        reset_arm_sub = nh_.subscribe("/z1_gazebo/reset_arm_pose",10,&UnitreeRos::ResetArmPoseCb,this);
        motion_plan_sub = nh_.subscribe("/motion_plan", 10, &UnitreeRos::MotionPlanCb, this);
        cartesian_control_sub = nh_.subscribe("/arm_pose_command",10,&UnitreeRos::CartesianControl,this); 
        gripper_as.start();
    }

    ~UnitreeRos() {
        // Stop the publishing thread if it's running
        if (publisher_thread.joinable()) {
            isPublishing = false;
            publisher_thread.join();
        }
    }

    void GripperMotionCb(const anytree_msgs::gripperCommandGoalConstPtr &goal) {
        //Get the targetQ
        std::cout << "Received action goal" << std::endl;
        double targetQ = goal->targetQ;
        //Check if the targetQ is within limits -1.57 ??

        //Send the targetQ to gripper
        sendGripperCommand(targetQ);
        //Check if the gripper reached target with some tolerance
        double tolerance = 0.4; //This is currenly not working as intended
        double Q = arm.lowstate->getGripperQ();
        if (Q > targetQ - tolerance && Q < targetQ + tolerance) {
            result_.result = true;
            gripper_as.setSucceeded(result_);
        } else {
            result_.result = false;
            gripper_as.setAborted(result_);
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
        Eigen::Map<const Eigen::VectorXd> positions_map(motion_plan.positions.data(), arm_dof);
        Eigen::Map<const Eigen::VectorXd> velocities_map(motion_plan.velocities.data(), arm_dof);
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
        UNITREE_ARM::Timer timer(dt);
        //Timer timer(control_frequency);
        for(int i(0); i<duration; i++){
            arm.q =  initQ * (1-i/duration) + targetQ * (i/duration);
            arm.qd = (targetQ - initQ) / (duration * dt);
            arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
            //gripperQ = -0.001;
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            //setGripperCmd(gripperQ, gripperW, gripperTau);
            arm.sendRecv();
            timer.sleep();
        }
        arm.sendRecvThread->start();
    }

    void CartesianControl(const trajectory_msgs::JointTrajectoryPoint& msg) {
        Vec6 posture; //Vector to store posture data
        double joint_speed = 1.0; //Max joint speed while moving
        double gripperPos = 0.0;
        //Set fsm state the joint control
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
        //Check if the incoming message includes gripper pos
        if (static_cast<int>(msg.positions.size()) == arm_dof) {
            posture << msg.positions[0],msg.positions[1],msg.positions[2],msg.positions[3],msg.positions[4],msg.positions[5];
            std::cout << posture.transpose() << std::endl;
            arm.MoveJ(posture, gripperPos, joint_speed);
        } else{
            if(static_cast<int>(msg.positions.size()) == arm_dof+1){
                Eigen::Map<const Vec6> posture(msg.positions.data(), arm_dof);
                gripperPos = msg.positions[arm_dof];
                arm.MoveJ(posture,gripperPos,joint_speed);
            }
            else {
                ROS_WARN("Invalid message type, can't move arm");
            }
        }
        //Return to lowcmd mode
        arm.setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
        arm.backToStart();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
    }

    void SetupArmSim() {
        //Start communications with arm controller
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        // SIM PDs, does not work in HW
        std::vector<double> KP = {100,150,150,100,75,50};
        std::vector<double> KD = {500,500,500,500,500,500};

        //Set control gains
        arm.lowcmd->setControlGain(KP,KD);
        //Sets up the publishers
        filtered_state_pub = nh_.advertise<sensor_msgs::JointState>("z1_gazebo/joint_states_filtered",10); 
        // HW PDs
        //arm.lowcmd->setControlGain();
    }

    void SetupArmHW() {
        //Start communications with the controller
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        //Set PD gains for HW
        arm.lowcmd->setControlGain();
        //Set the publishers
        filtered_state_pub = nh_.advertise<sensor_msgs::JointState>("z1_gazebo/joint_states_filtered",10); //For motion planner
        state_pub = nh_.advertise<sensor_msgs::JointState>("z1_gazebo/joint_states",10);   
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

    void SetJointStateMsg() {
        joint_state_msg.position.resize(arm_dof+1);
        joint_state_msg.velocity.resize(arm_dof+1);
        joint_state_msg.effort.resize(arm_dof+1);
        joint_state_msg.name = {"joint1","joint2","joint3","joint4","joint5","joint6","jointGripper"};
    }

private:
    void publishLoop() {
        ros::Rate rate(50); //50Hz
        while (ros::ok() && isPublishing) {
            //std::unique_lock<std::mutex> lock(lowstate_mutex);
            //Get the state values for the arm
            Vec6 arm_position = arm.lowstate->getQ();
            Vec6 arm_velocity = arm.lowstate->getQd();
            Vec6 arm_torques = arm.lowstate->getTau();
            //lock.unlock();
            Vec6 v_filtered = velocity_filter.filter(arm_velocity);
            // Directly assign values using Eigen::Map
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.position.data(), arm_dof) = arm_position;
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.velocity.data(), arm_dof) = v_filtered;
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.effort.data(),arm_dof) = arm_torques;
            //Get the state values for the gripper
            joint_state_msg.position[arm_dof] = arm.lowstate->getGripperQ();
            joint_state_msg.velocity[arm_dof] = arm.lowstate->getGripperQd();
            joint_state_msg.effort[arm_dof] = arm.lowstate->getGripperTau();
            //Set message time stamp
            joint_state_msg.header.stamp = ros::Time::now();

            //Publish message 
            if (isSim) {
                filtered_state_pub.publish(joint_state_msg);
            } else {
                state_pub.publish(joint_state_msg);
                filtered_state_pub.publish(joint_state_msg);
            }
            rate.sleep();
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_as;
    ros::Publisher state_pub;
    ros::Publisher filtered_state_pub;
    ros::Subscriber motion_plan_sub;
    ros::Subscriber cartesian_control_sub;
    ros::Subscriber reset_arm_sub;
    actionlib::SimpleActionServer<anytree_msgs::gripperCommandAction> gripper_as;
    anytree_msgs::gripperCommandResult result_;

    int arm_dof = 6;
    double dt;

    LowPassFilter<Eigen::VectorXd> velocity_filter; //LowPassFilter for velocity readings
    UNITREE_ARM::unitreeArm arm;
    trajectory_msgs::JointTrajectoryPoint motion_plan;

    std::thread publisher_thread;
    bool isPublishing;
    bool isSim;
    std::mutex lowstate_mutex;

    sensor_msgs::JointState joint_state_msg;

};