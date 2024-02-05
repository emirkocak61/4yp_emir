#pragma once

/*
    A class to represent anytree robot model
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <anytree_control/LowPassFilter.hpp>
#include <thread>

class RobotInterface {
public:
    RobotInterface(const double& dt) : arm(true), r(1/dt), velocity_filter(0.167,Eigen::VectorXd::Zero(arm_dof)) {
        this->dt = dt;
        robot_name = "anytree";
        motion_plan_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan", 10);
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        std::vector<double> KP = {200,300,300,200,150,100};
        std::vector<double> KD = {1000,1000,1000,1000,1000,1000};
        arm.lowcmd->setControlGain(KP, KD);
    }
    ~RobotInterface() {}
    
    void publishMotionPlan(const Eigen::VectorXd motion_plan) {
        UNITREE_ARM::Timer publisher_timer(dt);
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.reserve(robot_dof);
        //Convert from eigen vector to std vector
        for (int i = 0; i < 12; i++) {trajectory_point.positions[i] = motion_plan(i);}
        ros::Duration duration;
        trajectory_point.time_from_start = duration.fromSec(dt);
        trajectory_msg.points.push_back(trajectory_point);
        motion_plan_publisher.publish(trajectory_msg);
        publisher_timer.sleep();
    }

    void publishArmPlan(const Eigen::VectorXd motion_plan) {
        arm.sendRecvThread->shutdown();
        Vec6 targetQ = motion_plan.segment(motion_plan.size() - arm_dof,arm_dof); 
        Vec6 initQ = arm.lowstate->getQ();
        double duration = 10; 
        UNITREE_ARM::Timer timer(0.002); //This is the default control frequency of 500Hz
        //Timer timer(control_frequency);
        for(int i(0); i<duration; i++){
            arm.q = initQ * (1-i/duration) + targetQ * (i/duration);
            arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
            arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
            //std::cout << "Computed Torques: " << arm.tau << std::endl;
            //gripperQ = -0.001;
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            //setGripperCmd(gripperQ, gripperW, gripperTau);
            arm.sendRecv();
            timer.sleep();
        }
        arm.sendRecvThread->start();
    }


    //Returns the robots state in terms of arm position and base position and orientation
    Eigen::VectorXd GetRobotState() {
        Eigen::VectorXd robot_state(robot_dof * 2);
        std::unique_ptr<Eigen::VectorXd> base_pose = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(base_dof));
        std::unique_ptr<Eigen::VectorXd> base_twist = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(base_dof));

        Eigen::VectorXd arm_position = arm.lowstate->getQ();
        Eigen::VectorXd arm_velocity = arm.lowcmd->getQd();
        GetBaseState(base_pose, base_twist);
        robot_state << (*base_pose), arm_position, (*base_twist), arm_velocity;
        return robot_state;
    }
    
    void GetBaseState(std::unique_ptr<Eigen::VectorXd>& base_pose,std::unique_ptr<Eigen::VectorXd>& base_twist) {
        
        std::thread pose_thread(getBasePose, std::ref(base_pose));
        std::thread twist_thread(getBaseTwist,std::ref(base_twist));
        pose_thread.join();
        twist_thread.join();
    }

    void getBaseTwist(std::unique_ptr<Eigen::VectorXd>& base_twist) {
        geometry_msgs::TwistWithCovarianceStampedConstPtr base_twist_ptr;
        base_twist_ptr =  ros::topic::waitForMessage<geometry_msgs::TwistWithCovarianceStamped>("/state_estimator/twist");
        (*base_twist)(0) = base_twist_ptr->twist.twist.linear.x;
        (*base_twist)(1) = base_twist_ptr->twist.twist.linear.y;
        (*base_twist)(2) = base_twist_ptr->twist.twist.linear.z;
        (*base_twist)(3) = base_twist_ptr->twist.twist.angular.x;
        (*base_twist)(4) = base_twist_ptr->twist.twist.angular.y;
        (*base_twist)(5) = base_twist_ptr->twist.twist.angular.z;
    }

    void getBasePose(std::unique_ptr<Eigen::VectorXd>& base_pose) {
        geometry_msgs::PoseWithCovarianceStampedConstPtr base_pose_ptr;
        base_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/state_estimator/pose_in_odom");
        //Extract translation components
        (*base_pose)(0) = base_pose_ptr->pose.pose.position.x;
        (*base_pose)(1) = base_pose_ptr->pose.pose.position.y;
        (*base_pose)(2) = base_pose_ptr->pose.pose.position.z;

        tf2::Quaternion q(
            base_pose_ptr->pose.pose.orientation.x,
            base_pose_ptr->pose.pose.orientation.y,
            base_pose_ptr->pose.pose.orientation.z,
            base_pose_ptr->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll,pitch,yaw);

        //Set rotation components
        (*base_pose)(3) = roll;
        (*base_pose)(4) = pitch;
        (*base_pose)(5) = yaw;
    }
    
    std::string robot_name;

protected:
    ros::NodeHandle nh_;
    ros::Publisher motion_plan_publisher;
    ros::Rate r;
    double dt; //Control frequency 
    int robot_dof = 12; //Base + arm
    int base_dof = 6;
    int arm_dof = 6;

    UNITREE_ARM::unitreeArm arm;
    LowPassFilter<Eigen::VectorXd> velocity_filter; 
    

};
