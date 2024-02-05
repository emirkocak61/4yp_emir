#pragma once

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <anytree_control/LowPassFilter.hpp>
#include <mutex>

class UnitreeRos {
public:
    UnitreeRos() : 
    arm(true), 
    velocity_filter(0.167,Eigen::VectorXd::Zero(arm_dof)), isPublishing(false) {
        SetupArm();
        targetQdd = Eigen::VectorXd::Zero(arm_dof);
        motion_plan_sub = nh_.subscribe("/motion_plan", 10, &UnitreeRos::MotionPlanCb, this);
        state_publisher = nh_.advertise<sensor_msgs::JointState>("/z1_joint_states",10);    
    }
    ~UnitreeRos() {
        // Stop the publishing thread if it's running
        if (publisher_thread.joinable()) {
            isPublishing = false;
            publisher_thread.join();
        }
    }

    void MotionPlanCb(const trajectory_msgs::JointTrajectory& msg) {
        motion_plan = msg.points[0];
        // Use Eigen::Map to directly map the positions to an Eigen::VectorXd
        Eigen::Map<const Eigen::VectorXd> positions_map(motion_plan.positions.data(), arm_dof);
        Eigen::Map<const Eigen::VectorXd> velocities_map(motion_plan.velocities.data(), arm_dof);
        Eigen::Map<const Eigen::VectorXd> acceleration_map(motion_plan.accelerations.data(), arm_dof);
        arm.q = positions_map;
        arm.qd = velocities_map;
        targetQdd = acceleration_map;
        
        sendArmCommand();
    }

    void sendArmCommand() {
        arm.sendRecvThread->shutdown();
        //std::unique_lock<std::mutex> lock(lowstate_mutex);
        //Vec6 initQ = arm.lowstate->getQ();
        //lock.unlock();
        //double duration = 10; //Use 5 instead of 10 to compensate for any delays 
        //UNITREE_ARM::Timer timer(arm._ctrlComp->dt); //This is the default control frequency of 500Hz
        //Timer timer(control_frequency);
        //for(int i(0); i<duration; i++){
        //arm.q =  targetQ;
        //arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        //gripperQ = -0.001;
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        //setGripperCmd(gripperQ, gripperW, gripperTau);
        arm.sendRecv();
        //timer.sleep();
        //}
        arm.sendRecvThread->start();
    }

    void SetupArm() {
        //Start communications with arm controller
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        std::vector<double> KP = {100,150,150,100,75,50};
        std::vector<double> KD = {1000,1000,1000,1000,1000,1000};
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
    ros::Publisher state_publisher;
    int arm_dof = 6;

    LowPassFilter<Eigen::VectorXd> velocity_filter; //LowPassFilter for velocity readings
    UNITREE_ARM::unitreeArm arm;
    trajectory_msgs::JointTrajectoryPoint motion_plan;
    Eigen::VectorXd targetQdd;

    std::thread publisher_thread;
    bool isPublishing;
    std::mutex lowstate_mutex;

};