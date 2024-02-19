#pragma once

/*
    A class to represent anytree robot model
*/
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <thread>

class RobotInterface {
public:
    RobotInterface(const double& dt) :  r(50) {
        this->dt = dt;
        robot_state_publisher = nh_.advertise<std_msgs::Float64MultiArray>("/robot_state", 10);
        arm_state_sub = nh_.subscribe("z1_joint_states",10,&RobotInterface::armStateCallback, this);
        base_twist_sub = nh_.subscribe("/state_estimator/twist",10,&RobotInterface::BaseTwistCb,this);
        base_pose_sub = nh_.subscribe("state_estimator/pose_in_odom",10,&RobotInterface::BasePoseCb,this);
        //Initialize all the vectors for robots state
        robot_state = Eigen::VectorXd::Zero(robot_dof*2);
        robot_state_msg.data.resize(robot_dof * 2);
        arm_state = Eigen::VectorXd::Zero(arm_dof*2);
        base_pose = Eigen::VectorXd::Zero(base_dof);
        base_twist = Eigen::VectorXd::Zero(base_dof);
        //Start the update thread for robot pose
        startUpdating();
    }

    ~RobotInterface() { stopUpdating();}
    /*
    void publishMotionPlan(const Eigen::VectorXd& motion_plan) {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.points.reserve(1);
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(robot_dof);
        trajectory_point.velocities.resize(robot_dof);
        //Now we have to convert the Eigen::VectorXds into std::vector
        //We can use Eigen::Map to directly map trajectory_point.positions to Eigen::VectorXd without copying
        Eigen::Map<Eigen::VectorXd>(trajectory_point.positions.data(), trajectory_point.positions.size()) = motion_plan.head(robot_dof);
        Eigen::Map<Eigen::VectorXd>(trajectory_point.velocities.data(), trajectory_point.velocities.size()) = motion_plan.tail(robot_dof);
        trajectory_msg.points.push_back(trajectory_point);
        motion_plan_publisher.publish(trajectory_msg);
    }
    */

    void armStateCallback(const sensor_msgs::JointState::ConstPtr& arm_msg) {
        std::lock_guard<std::mutex> lock(arm_mutex);
        Eigen::Map<Eigen::VectorXd>(arm_state.data(), 6) = Eigen::VectorXd::Map(&arm_msg->position[0], 6);
        Eigen::Map<Eigen::VectorXd>(arm_state.data() + 6, 6) = Eigen::VectorXd::Map(&arm_msg->velocity[0], 6);
    }

    void BaseTwistCb (const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg) {
        std::lock_guard<std::mutex> lock(base_twist_mutex);
        base_twist << twist_msg->twist.twist.linear.x,
                      twist_msg->twist.twist.linear.y,
                      twist_msg->twist.twist.linear.z,
                      twist_msg->twist.twist.angular.x,
                      twist_msg->twist.twist.angular.y,
                      twist_msg->twist.twist.angular.z;
    }

    void BasePoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
        // Map the first three elements of base_pose to the position
        std::lock_guard<std::mutex> lock(base_pose_mutex);
        Eigen::Map<Eigen::Vector3d>(base_pose.data()) = Eigen::Vector3d(
            pose_msg->pose.pose.position.x,
            pose_msg->pose.pose.position.y,
            pose_msg->pose.pose.position.z
        );

        // Convert Quaternion to Roll-Pitch-Yaw and map to the last three elements of base_pose
        tf2::Quaternion q(
            pose_msg->pose.pose.orientation.x,
            pose_msg->pose.pose.orientation.y,
            pose_msg->pose.pose.orientation.z,
            pose_msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        Eigen::Map<Eigen::Vector3d>(base_pose.data() + 3) = Eigen::Vector3d(roll, pitch, yaw);
    }

    void updateRobotState() {
        while(ros::ok() && keepUpdating) {
            {
            std::lock_guard<std::mutex> lock_arm(arm_mutex);
            std::lock_guard<std::mutex> lock_base_pose(base_pose_mutex);
            std::lock_guard<std::mutex> lock_base_twist(base_twist_mutex);
            //Positions
            robot_state.head(base_dof) = base_pose;
            robot_state.segment(base_dof,arm_dof) = arm_state.head(6);
            //Velocities
            robot_state.segment(robot_dof,base_dof) = base_twist;
            robot_state.tail(arm_dof) = arm_state.tail(6);
            Eigen::Map<Eigen::VectorXd>(robot_state_msg.data.data(),robot_state_msg.data.size()) = robot_state;
            robot_state_publisher.publish(robot_state_msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        }
    }
    
    Eigen::VectorXd getRobotState() {
        std::lock_guard<std::mutex> lock(robot_state_mutex);
        return robot_state;
    }
    

    //Function that starts the thread for updating robot state
    void startUpdating() {
        if (!keepUpdating) {
        keepUpdating = true;
        updateThread = std::thread(&RobotInterface::updateRobotState, this);
        }
    }
    //Function that stops the thread for updating robot state
    void stopUpdating() {
    if (keepUpdating) {
        keepUpdating = false;
        if (updateThread.joinable()) {
            updateThread.join();
        }
    }
}
    int robot_dof = 12; //Base + arm
    int base_dof = 6;
    int arm_dof = 6;

protected:
    ros::NodeHandle nh_;
    ros::Publisher robot_state_publisher;
    ros::Subscriber arm_state_sub;
    ros::Subscriber base_pose_sub;
    ros::Subscriber base_twist_sub;
    ros::Rate r;

    //Vectors related to robots state
    std_msgs::Float64MultiArray robot_state_msg;
    Eigen::VectorXd robot_state;
    Eigen::VectorXd arm_state;
    Eigen::VectorXd base_pose;
    Eigen::VectorXd base_twist; 

    //Mutexes to protect data
    std::mutex robot_state_mutex;
    std::mutex arm_mutex;
    std::mutex base_pose_mutex;
    std::mutex base_twist_mutex;

    std::thread updateThread;
    bool keepUpdating = false;

    double dt; //Control frequency 

};
