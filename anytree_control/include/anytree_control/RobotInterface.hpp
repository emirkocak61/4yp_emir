#pragma once

/*
    This class is used to combine all the state estimations from various topics into a one robot state vector
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
        arm_state_sub = nh_.subscribe("z1_gazebo/joint_states_filtered",10,&RobotInterface::ArmStateCb, this);
        base_twist_sub = nh_.subscribe("/state_estimator/twist",10,&RobotInterface::BaseTwistCb,this);
        base_pose_sub = nh_.subscribe("state_estimator/pose_in_odom",10,&RobotInterface::BasePoseCb,this);
        //Initialize all the vectors for robots state
        robot_state = Eigen::VectorXd::Zero(robot_dof*2);
        robot_state_msg.data.resize(robot_dof * 2);
        arm_position = Eigen::VectorXd::Zero(arm_dof);
        arm_velocity = Eigen::VectorXd::Zero(arm_dof);
        base_pose = Eigen::VectorXd::Zero(base_dof);
        base_twist = Eigen::VectorXd::Zero(base_dof);
        //Start the update thread for robot pose
        startUpdating();
    }

    ~RobotInterface() { stopUpdating();}
    

    void ArmStateCb(const sensor_msgs::JointStateConstPtr &state) {
        std::lock_guard<std::mutex> lock(arm_mutex);
        Eigen::Map<const Eigen::VectorXd> posMap(state->position.data(), arm_dof);
        Eigen::Map<const Eigen::VectorXd> velMap(state->velocity.data(), arm_dof);
        arm_position = posMap;
        arm_velocity = velMap;
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
        ros::Rate rate(50); //50Hz
        while(ros::ok() && keepUpdating) {
            {
            std::lock_guard<std::mutex> lock_arm(arm_mutex);
            std::lock_guard<std::mutex> lock_base_pose(base_pose_mutex);
            std::lock_guard<std::mutex> lock_base_twist(base_twist_mutex);
            //Positions
            robot_state.head(base_dof) = base_pose;
            robot_state.segment(base_dof,arm_dof) = arm_position;
            //Velocities
            robot_state.segment(robot_dof,base_dof) = base_twist;
            robot_state.tail(arm_dof) = arm_velocity;
            Eigen::Map<Eigen::VectorXd>(robot_state_msg.data.data(),robot_state_msg.data.size()) = robot_state;
            robot_state_publisher.publish(robot_state_msg);
            }
            rate.sleep();
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
    Eigen::VectorXd arm_position;
    Eigen::VectorXd arm_velocity;
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
