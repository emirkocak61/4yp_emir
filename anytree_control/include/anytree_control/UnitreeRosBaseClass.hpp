#pragma once
/*
    The class that interfaces the unitree arm with ROS. This is an abstract class
    that includes the base functions to control the arm. Child classes must be derived
    for hardware and simulation. The base functions include the gripper and arm feedforward 
    control. The interface is designed to keep things in lowcmd mode
*/
#include <anytree_control/ExtendedKalmanFilter.hpp>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include "actionlib/server/simple_action_server.h"
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <anytree_msgs/gripperCommandAction.h>
#include <anytree_msgs/gripperCommandResult.h>
#include <anytree_msgs/moveArmAction.h>
#include <anytree_msgs/moveArmResult.h>
#include <mutex>
#include <random>

class UnitreeRosBaseClass {
public:

    UnitreeRosBaseClass() :
    arm(true),
    dt(0.002),
    isPublishing(false),
    gripper_as(nh_as,"gripperCommand_as",boost::bind(&UnitreeRosBaseClass::GripperMotionCb,this,_1),false),
    arm_as(nh_as,"/z1_gazebo/moveArm_as",boost::bind(&UnitreeRosBaseClass::MoveArmCb,this,_1),false) {
        //Start communications with the arm
        arm.sendRecvThread->start();
        SetJointStateMsg();
        InitSavedStates();
        LoadParam();
        motion_plan_sub = nh_.subscribe("/motion_plan", 10, &UnitreeRosBaseClass::MotionPlanCb, this);
        gripper_as.start();
        arm_as.start();
        distribution = std::normal_distribution<double>(0.0,0.01);
    }

    ~UnitreeRosBaseClass() {
        gripper_as.shutdown();
        arm_as.shutdown();
    }

    void MotionPlanCb(const trajectory_msgs::JointTrajectory& msg) {
        motion_plan = msg.points[0];
        // Use Eigen::Map to directly map the positions to an Eigen::VectorXd
        Eigen::Map<const Eigen::VectorXd> positions_map(motion_plan.positions.data()+base_dof, arm_dof);
        Eigen::Map<const Eigen::VectorXd> velocities_map(motion_plan.velocities.data()+base_dof, arm_dof);
        double duration = 10;
        arm.q = positions_map;
        arm.qd = velocities_map;
        sendArmCommand(positions_map, duration);
    }

    void GripperMotionCb(const anytree_msgs::gripperCommandGoalConstPtr &goal) {
        //Get the targetQ
        double targetQ = goal->targetQ;
        //Check if the targetQ is within limits -1.57 ??
        double duration = goal->duration;
        //Send the targetQ to gripper
        sendGripperCommand(targetQ,duration);
        //Check if the gripper reached target with some tolerance
        double tolerance = 0.4; //This is currenly not working as intended
        double Q = arm.lowstate->getGripperQ();
        //Set the result variable
        anytree_msgs::gripperCommandResult result_;
        if (Q > targetQ - tolerance && Q < targetQ + tolerance) {
            result_.result = true;
            gripper_as.setSucceeded(result_);
        } else {
            result_.result = false;
            gripper_as.setAborted(result_);
        }
    }

    void MoveArmCb(const anytree_msgs::moveArmGoalConstPtr &goal) {

        //Create  a result object
        anytree_msgs::moveArmResult result_;
        //Create necessary objects
        Vec6 targetQ;
        double duration;
        //Check which part of the goal is populated and act accordingly
        //First check if both are populated
        if (!goal->label.empty() && !goal->posture.empty()) {
            ROS_WARN("Please choose one of the control modes");
            result_.result = false;
            arm_as.setAborted(result_);
            return;

        } else if (!goal->label.empty()) {
            std::string label = goal->label;
            duration = goal->duration;
            targetQ = arm_states[label];
        } else if (!goal->posture.empty()) {
            duration = goal->duration;
            Eigen::Map<const Vec6> posture(goal->posture.data(),goal->posture.size());
            //Convert the posture vector into a homomat
            HomoMat mat = UNITREE_ARM::postureToHomo(posture);
            //Create the solution object
            Vec6 solution;
            //Solve inverse kinematics to get desired joint configuration
            Vec6 q_quess; //Vector to pass as initial guess to the IK solver
            q_quess << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
            if (!arm._ctrlComp->armModel->inverseKinematics(mat,q_quess,solution,true)){
                ROS_WARN("Inverse kinematic solution does not exist");
                result_.result = false;
                arm_as.setAborted(result_);
                return;
            };
            //Assign the solution to targetQ object
            targetQ = solution;
        } else {
            //Return failure if nothing was provided
            result_.result = false;
            arm_as.setAborted(result_, "Neither label nor targetQ was provided");
            return;
        }
        sendArmCommand(targetQ,duration);
        //Assume the action was succesfull
        result_.result = true;
        arm_as.setSucceeded(result_);
    }

    void sendGripperCommand(const double targetQ,const double duration) {
        arm.sendRecvThread->shutdown();
        double initQ = arm.lowstate->getGripperQ();
        UNITREE_ARM::Timer timer(dt);
        for (int i(0); i < duration; i++) {
            //Set the gripper commands by linear interpolation
            arm.gripperQ = initQ * (1-i/duration) + targetQ * (i/duration);
            arm.gripperW = (targetQ - initQ) / (duration * dt);
            
            //Use the latest arm commands to keep the arm steady
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            //Set the gripper commands, assuming 0 torque
            arm.setGripperCmd(arm.gripperQ,arm.gripperW);
            arm.sendRecv();
            timer.sleep();
        }
        arm.sendRecvThread->start();
    }

    void sendArmCommand(const Eigen::VectorXd& targetQ,const double duration) {
        arm.sendRecvThread->shutdown();
        Vec6 initQ = arm.lowstate->getQ();
        UNITREE_ARM::Timer timer(dt);
        //Timer timer(control_frequency);
        Eigen::VectorXd noise = Eigen::VectorXd::Ones(6);
        for(int i(0); i<duration; i++){
            arm.q =  initQ * (1-i/duration) + targetQ * (i/duration); // + distribution(generator) * noise;
            arm.qd = (targetQ - initQ) / (duration * dt);
            arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
            //gripperQ = -0.001;
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            arm.setGripperCmd(arm.gripperQ, arm.gripperW);
            arm.sendRecv();
            timer.sleep();
        }
        arm.sendRecvThread->start();
    }

    void SetJointStateMsg() {
        joint_state_msg.position.resize(arm_dof+1);
        joint_state_msg.velocity.resize(arm_dof+1);
        joint_state_msg.effort.resize(arm_dof+1);
        joint_state_msg.name = {"joint1","joint2","joint3","joint4","joint5","joint6","jointGripper"};
    }

    void InitSavedStates() {
        Vec6 forward;
        Vec6 startFlat;
        forward << 0.0,1.5,-1.0,-0.54,0.0,0.0;
        startFlat << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0;
        arm_states["forward"] = forward;
        arm_states["startFlat"] = startFlat;
    }
    //Loads the necessary parameters from ros parameter server
    void LoadParam() {
        if(!nh_.getParam("base_dof", base_dof)){
            ROS_WARN("Failed to load base_dof parameter, assuming standalone arm");
            base_dof = 0;
        }
    }
    //Function that starts the state publisher loop in a separate thread
    void startPublishing() {
        if (!isPublishing) {
            isPublishing = true;
            publisher_thread = std::thread(&UnitreeRosBaseClass::publishLoop, this);
        }
    }
    //Function that joins the state publisher back and stops publishing
    void stopPublishing() {
        if (isPublishing) {
            isPublishing = false;
            if (publisher_thread.joinable()) {
                publisher_thread.join();
            }
        }
    }

    virtual void SetupArm() = 0; //Function that sets the arm related parameters

protected:
    virtual void publishLoop() = 0; //The function that implements the state publisher

    ros::NodeHandle nh_;
    ros::NodeHandle nh_as;
    UNITREE_ARM::unitreeArm arm;
    double dt;
    
    actionlib::SimpleActionServer<anytree_msgs::gripperCommandAction> gripper_as;
    actionlib::SimpleActionServer<anytree_msgs::moveArmAction> arm_as;
    ros::Subscriber motion_plan_sub;

    int arm_dof = 6;
    int base_dof;

    //LowPassFilter<Eigen::VectorXd> velocity_filter; //LowPassFilter for velocity readings
    
    trajectory_msgs::JointTrajectoryPoint motion_plan;

    std::thread publisher_thread;
    bool isPublishing;
    std::mutex lowstate_mutex;

    sensor_msgs::JointState joint_state_msg;
    std::map<std::string,Vec6> arm_states;

    EKF ekf; //Kalman filter

    std::default_random_engine generator; //Random endgine
    std::normal_distribution<double> distribution;
};