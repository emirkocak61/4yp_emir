/*
    Script to test the AICO solver for planning a trajectory for motion planning
*/
#pragma once

#include <exotica_core/exotica_core.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace exotica;

class TestAICO {
    typedef KDL::Frame Frame;
public:

    TestAICO() : rate(10) {
        dt = 0.1;
        alpha = 0.8;
        q = Eigen::VectorXd::Zero(robot_dof);
        qd = Eigen::VectorXd::Zero(robot_dof);
        arm_state = Eigen::VectorXd::Zero(arm_dof);
        base_pose = Eigen::VectorXd::Zero(base_dof);
        robot_state = Eigen::VectorXd::Zero(robot_dof);
        rest_pose = LookupBasePose();
        mp_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan",10);
        arm_state_subsciber = nh_.subscribe("/z1_gazebo/joint_states_filtered",10,&TestAICO::ArmStateCb,this);
        base_state_subscriber = nh_.subscribe("/state_estimator/pose_in_odom",10,&TestAICO::BaseStateCb,this);
    }
    ~TestAICO() {}

    void SetupProblem(const std::string& config_path) {

        Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("test_node")));
        solver = XMLLoader::LoadSolver(config_path);
        //Get Problem
        PlanningProblemPtr planning_problem = solver->GetProblem(); //This returns a generic problem
        //Downcast the pointer to a UnconstrainedTimeIndexedProblem pointer
        problem = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(planning_problem);
        //Get exotica scene
        scene = problem->GetScene();
        robot_state << base_pose , arm_state;
        scene->SetModelState(robot_state);
        problem->Update(scene->GetControlledState(),0);
        scene->GetKinematicTree().PublishFrames();
        SetJointLimits();
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);      
        t = 0.0;
        std::cout << "Problem all set up" << std::endl;   
    }

    void InitRobotPose() {
        robot_state << base_pose , arm_state;
        scene->SetModelState(robot_state);
        problem->Update(scene->GetControlledState(),0);
        scene->GetKinematicTree().PublishFrames();
        SetJointLimits();
    }

    Eigen::VectorXd LookupBasePose() {
        boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> pose_message;
        pose_message = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/state_estimator/pose_in_odom", nh_, ros::Duration(1.0));
        Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);

        //Extract the translation component
        pose(0) = pose_message->pose.pose.position.x;
        pose(1) = pose_message->pose.pose.position.y;
        pose(1) = pose_message->pose.pose.position.z;

        //Convert rotation components into RPY
         tf2::Quaternion q(
            pose_message->pose.pose.orientation.x,
            pose_message->pose.pose.orientation.y,
            pose_message->pose.pose.orientation.z,
            pose_message->pose.pose.orientation.w
         );
         tf2::Matrix3x3 m(q);
         double roll, pitch, yaw;
         m.getRPY(roll,pitch,yaw);

         //Set rotation components
         pose(3) = roll;
         pose(4) = pitch;
         pose(5) = yaw;
         return pose;
    }

    void PublishMotionPlan() {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.points.reserve(1);
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(robot_dof);
        trajectory_point.velocities.resize(robot_dof); 
        //Convert from Eigen::VectorXd to std::vector
        // Use Eigen::Map to directly map trajectory_point.positions to an Eigen::VectorXd
        Eigen::Map<Eigen::VectorXd>(trajectory_point.positions.data(), trajectory_point.positions.size()) = q;
        Eigen::Map<Eigen::VectorXd>(trajectory_point.velocities.data(),trajectory_point.velocities.size()) = qd;
        trajectory_msg.points.push_back(trajectory_point);
        mp_publisher.publish(trajectory_msg);  
    }

    void ArmStateCb(const sensor_msgs::JointStateConstPtr &state) {
        std::lock_guard<std::mutex> lock(state_mutex);
        Eigen::Map<const Eigen::VectorXd> tempMap(state->position.data(), state->position.size());
        arm_state = tempMap;
    }

    void BaseStateCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
        std::lock_guard<std::mutex> lock(base_mutex);
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

    void SetupGoal(const Frame target_frame) {
        scene->AttachObjectLocal("Target","", target_frame);
        int T = problem->GetT();
        std::cout << "Problem T: " << T << std::endl;
        Eigen::VectorXd goal(6);
        double rho;
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        for (int t(0); t < T; t++) {
            rho = pow(alpha, t) * 1e4;
            problem->SetGoal("Position",goal,t);
            problem->SetRho("Position",rho,t);
        }
        //problem->SetGoal("Position",goal,-1);
        //problem->SetRho("Position", 1e3,-1);
        std::cout << "Problem goal set" << std::endl;
    }

    void Algorithm() {
        problem->SetStartTime(0);
        robot_state << base_pose , arm_state;
        problem->SetStartState(robot_state);
        //Create a solution container
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        //Solve problem
        solver->Solve(*solution);
        //Calculate velocities
        Eigen::VectorXd qnew(12);
        qnew = solution->row(1);
        qd = (qnew-q)/(dt);
        q = qnew;
        std::cout << "q: " << q.transpose() << std::endl;
        //Update problem
        problem->Update(q.transpose(),1);
        
        scene->GetKinematicTree().PublishFrames();        
    }

    void SetJointLimits() {  
        std::map<std::string,double> model_state;
        model_state["world_joint/trans_x"] = rest_pose(0);
        model_state["world_joint/trans_y"] = rest_pose(1);
        model_state["world_joint/trans_z"] = rest_pose(2);
        model_state["world_joint/rot_x"] = rest_pose(3);
        model_state["world_joint/rot_y"] = rest_pose(4);
        model_state["world_joint/rot_z"] = rest_pose(5);
        scene->SetModelState(model_state); //Sets the state of the base in EXOTica

        Eigen::VectorXd global_joint_limits_upper, global_joint_limits_lower;
        global_joint_limits_upper = scene->GetKinematicTree().GetJointLimits().col(1);
        global_joint_limits_lower = scene->GetKinematicTree().GetJointLimits().col(0);

        for (int i = 0; i < base_dof; i++) {
            global_joint_limits_upper(i) = (global_joint_limits_upper(i) +
                                            rest_pose(i) + joint_limit_tolerance);
            global_joint_limits_lower(i) = (global_joint_limits_lower(i) +
                                            rest_pose(i) - joint_limit_tolerance);
        }

        //Set joint limits in EXOTica scene
        scene->GetKinematicTree().SetJointLimitsUpper(global_joint_limits_upper);
        scene->GetKinematicTree().SetJointLimitsLower(global_joint_limits_lower);
    }


    void Run() {
        t=0;
        int i = 0;
        while (t < time_limit) { 
            std::cout << "Iteration: " << i << std::endl;
            //Run one iteration
            Algorithm();
            //Publish the motion plan
            PublishMotionPlan();
            double error = GetError();
            std::cout << "Error: " << error << std::endl;
            ros::spinOnce();
            rate.sleep();
            t += dt;
            std::cout << "t: " << t << std::endl;
            i += 1;
        }
    }

    virtual void PerformMotion() {}

    double GetError() {
        return problem->GetScalarTaskCost(0);
    }


protected:
    int arm_dof = 6;
    int base_dof = 6;
    int robot_dof = 12;
    double time_limit = 50.0;
    double joint_limit_tolerance = 0.0001;
    double alpha; //Forgetting factor for the cost
    //EXOTica related objects
    MotionSolverPtr solver;
    UnconstrainedTimeIndexedProblemPtr problem;
    ScenePtr scene;
    Eigen::VectorXd q; //Vector to store joint positions
    Eigen::VectorXd qd; //Vector to store joint velocities
    std::mutex state_mutex;
    std::mutex base_mutex; //Mutex to protect base pose data
    Eigen::VectorXd rest_pose;
    Eigen::VectorXd robot_state;
    Eigen::VectorXd arm_state;
    Eigen::VectorXd base_pose;

    ros::NodeHandle nh_;
    ros::V_string joint_names;
    ros::Rate rate;
    ros::Publisher mp_publisher;
    ros::Subscriber arm_state_subsciber;
    ros::Subscriber base_state_subscriber;

    double dt; //Problem time step
    double t; //Variable to keep track the problem time
};