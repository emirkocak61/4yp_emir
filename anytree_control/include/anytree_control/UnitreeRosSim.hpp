#pragma once
/*
    The child class of UnitreeRosBaseClass specific for simulation implementation. 
*/

#include <anytree_control/UnitreeRosBaseClass.hpp>
#include <anytree_control/LowPassFilter.hpp>


class UnitreeRosSim : public UnitreeRosBaseClass {
public:
    UnitreeRosSim() : UnitreeRosBaseClass(),
    velocity_filter(0.167,Eigen::VectorXd::Zero(arm_dof)),
    torque_filter(0.167,Eigen::VectorXd::Zero(arm_dof)) {
        SetupArm();
    };

    //Sets up the arm with sim specific parameters
    void SetupArm() override {
        //Set arm into lowcmd mode 
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        // Set sim PD gains
        std::vector<double> KP = {100,150,150,100,75,50};
        std::vector<double> KD = {500,500,500,500,500,500};

        //Set control gains
        arm.lowcmd->setControlGain(KP,KD);
        //Sets up the publishers
        state_pub = nh_.advertise<sensor_msgs::JointState>("z1_gazebo/joint_states_filtered",10);   
    }

    void publishLoop() override {
        ros::Rate rate(50); //50Hz
        while (ros::ok() && isPublishing) {
            //std::unique_lock<std::mutex> lock(lowstate_mutex);
            //Get the state values for the arm
            Vec6 arm_position = arm.lowstate->getQ();
            Vec6 arm_velocity = arm.lowstate->getQd();
            Vec6 arm_torques = arm.lowstate->getTau();
            //lock.unlock();
            Vec6 v_filtered = velocity_filter.filter(arm_velocity);
            Vec6 tau_filtered = torque_filter.filter(arm_torques);
            // Directly assign values using Eigen::Map
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.position.data(), arm_dof) = arm_position;
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.velocity.data(), arm_dof) = v_filtered;
            Eigen::Map<Eigen::VectorXd>(joint_state_msg.effort.data(),arm_dof) = tau_filtered;
            //Get the state values for the gripper
            joint_state_msg.position[arm_dof] = arm.lowstate->getGripperQ();
            joint_state_msg.velocity[arm_dof] = arm.lowstate->getGripperQd();
            joint_state_msg.effort[arm_dof] = arm.lowstate->getGripperTau();
            //Set message time stamp
            joint_state_msg.header.stamp = ros::Time::now();

            //Publish message 
            state_pub.publish(joint_state_msg);
            rate.sleep();
        }
    }

private:
    
    LowPassFilter<Eigen::VectorXd> velocity_filter; //LowPassFilter for velocity readings
    LowPassFilter<Eigen::VectorXd> torque_filter; //LowPassFilter for torque readings
    ros::Publisher state_pub;

};