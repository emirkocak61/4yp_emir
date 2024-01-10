/**
 * Script to test mpc based algorithms on RVIZ
*/
#pragma once

#include <exotica_core/exotica_core.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
using namespace exotica;

class TestAlgorithm {
typedef geometry_msgs::Pose Pose;

public:
    //Constructor
    TestAlgorithm() : r(1/dt) {
        pub = nh_pub.advertise<sensor_msgs::JointState>("/joint_states",10);
    }
    //Destructor
    ~TestAlgorithm() {
        //This is important to avoid memory leaks
        solver.reset();
    }

    //Assumes the input is in the form xyz+rpy
    Pose GetPoseFromEuler(Eigen::VectorXd& euler) {
        Pose pose;

        //Get the translation components
        pose.position.x = euler(0);
        pose.position.y = euler(1);
        pose.position.z = euler(2);

        //Define a quaternion
        tf2::Quaternion q;
        q.setRPY(euler(3),euler(4),euler(5));

        //Set the orientation
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }

    //Sets up the exotica problem
    void SetupProblem(const std::string& config_path,
                      const Eigen::VectorXd& start_state,
                      const Eigen::VectorXd& target_pose) {
        //Setup EXOTica ROS
        Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("test_node")));

        solver = XMLLoader::LoadSolver(config_path);
        PlanningProblemPtr planning_problem = solver->GetProblem();
        //Static cast the pointer to a DynamicTimeIndexedShootingProblem
        problem = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(planning_problem);
        //Get the EXOTica scene
        scene = problem->GetScene();
        scene->SetModelState(start_state);
        problem->Update(scene->GetControlledState(),0);
        scene->GetKinematicTree().PublishFrames();
        joint_names = scene->GetControlledJointNames();
        scene->AttachObjectLocal("Target","world_frame", target_pose);
        
        int T = problem->get_T();
        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::VectorXdRefConst goal_ref = goal;
        for (int t = 0; t < T; ++t) {
            problem->cost.SetGoal("Position", goal_ref, t);
            problem->cost.SetRho("Position",1e1,t);
            }
        problem->cost.SetGoal("Position", goal_ref,T-1);
        problem->cost.SetRho("Position",1e3,T-1);
        
        q = problem->GetStartState();
        scene->SetModelState(q.head(scene->get_num_positions()));
        
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);        
        dt = 0.02;
        t = 0.0;
        std::cout << "Problem all set up" << std::endl;
    }

    void SetGoal(const Eigen::VectorXd& target_pose) {
        int T = problem->get_T();
        Eigen::VectorXdRefConst goal_ref = target_pose;
        problem->cost.SetGoal("Position", goal_ref,T-1);
    }
    

    void Algorithm() {
        problem->SetStartTime(t);
        problem->SetStartState(q);
        Eigen::MatrixXd solution;

        //Solve problem
        solver->Solve(solution);
        std::cout << "Problem Solved: " << solution.row(0) << std::endl;
        //Apply only the first step of the solution
        problem->Update(q,solution.row(0),0);

        //Update Joint Positions
        std::cout << "Updating problem state" << std::endl;
        q = problem->get_X(1);
        std::cout << "Problem state: " << q.transpose() << std::endl;
        scene->SetModelState(q.head(scene->get_num_positions()));
        std::cout << "Model state set" << std::endl;
        //PublishFramesToRVIZ(q);

        scene->GetKinematicTree().PublishFrames();
        std::cout << "Published Frames" << std::endl;
    }

    void RunAlgorithm(const double& time_limit) {
        while (t < time_limit) {
            Algorithm();
            r.sleep();
            t += dt;
            
        }

    }

    void PublishFramesToRVIZ(const Eigen::VectorXd& q) {
        sensor_msgs::JointState joint_states_msg;
        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name = joint_names;
        std::vector<double> positions;
        for (int i = 0; i < scene->get_num_positions(); i++) {
            positions[i] = q(i);
        }
        joint_states_msg.position = positions;
        pub.publish(joint_states_msg);
    }

private:
    ros::NodeHandle nh_pub;
    MotionSolverPtr solver; //EXOTica solver
    DynamicTimeIndexedShootingProblemPtr problem; //EXOTica planning problem
    ScenePtr scene; //EXOTica scene
    Eigen::VectorXd q; //Vector to store joint configurations
    ros::V_string joint_names;
    ros::Rate r; 
    ros::Publisher pub;
   

    double dt; //problem time step
    double t; //variable to keep track of time

};

