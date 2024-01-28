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
#include <chrono>
#include <memory>

using namespace exotica;

class TestAlgorithm {
typedef geometry_msgs::Pose Pose;

public:
    //Constructor
    TestAlgorithm() : r(1/dt) {
        pub = nh_pub.advertise<sensor_msgs::JointState>("/joint_states",10);
        isTraj = false;
         
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

    KDL::Frame GetFrameFromPose(const Pose &pose) {
        //Create a KDL Vector for the position
        KDL::Vector position(pose.position.x,pose.position.y,pose.position.z);
        //Create a KDL rotation from the quaternion
        KDL::Rotation rotation = KDL::Rotation::Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        );
        //Create a KDL frame using the position and rotation
        KDL::Frame frame(rotation,position);
        return frame;
    }

    //Sets up the exotica problem
    void SetupProblem(const std::string& config_path,
                      const Eigen::VectorXd& start_state,
                      const Pose target_pose) {
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
        KDL::Frame target_frame = GetFrameFromPose(target_pose);
        scene->AttachObjectLocal("Target","", target_frame);
        int T = problem->get_T();
        //Initialize the prevU object
        prevU = std::make_unique<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(T-1,6)); 

        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::VectorXdRefConst goal_ref = goal;
        for (int t = 0; t < T; ++t) {
            problem->cost.SetGoal("Position", goal_ref, t);
            problem->cost.SetRho("Position",1e1,t);
            }
        problem->cost.SetGoal("Position", goal_ref,T-1);
        problem->cost.SetRho("Position",1e4,T-1);
        
        q = problem->GetStartState();
        scene->SetModelState(q.head(scene->get_num_positions()));
        
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);        
        dt = 0.02;
        t = 0.0;
        std::cout << "Problem all set up" << std::endl;
    }

    
    Trajectory DefineTrajectory() {
    
        Eigen::MatrixXd trajectory;
        //Create trajectory
        trajectory = Eigen::MatrixXd::Zero(4,7); //time + xyz + rpy
        trajectory.row(0) <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0; //start
        trajectory.row(1) << 4.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0; //head on
        trajectory.row(2) << 6.0, 0.0, 0.0, 0.1, 0.0, 0.0, -1.5708; //rotate gripper
        trajectory.row(3) << 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5708; //grasp
        
        
        Trajectory traj_exotica(trajectory,1.0);
        return traj_exotica;
    }

    void Algorithm() {
        problem->SetStartTime(t);
        problem->SetStartState(q);
        //Set the previous solution as an initial guess
        problem->set_U(prevU->transpose());
        
        //Solve problem
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        auto start = std::chrono::high_resolution_clock::now();
        solver->Solve(*solution);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
        t_algorithm.push_back(duration);
        //Apply only the first step of the solution
        problem->Update(q,solution->row(0),0);
        //Print solution
        std::cout << solution->row(0) << std::endl;
        //Swap prevU and solution pointers for the next iteration
        prevU.swap(solution);

        //Update Joint Positions
        q = problem->get_X(1);
        //Print state at time t=1
        std::cout << q << std::endl;
        scene->SetModelState(q.head(scene->get_num_positions()));
        //PublishFramesToRVIZ(q);
        scene->GetKinematicTree().PublishFrames();

        //Output current cost
        double cost = problem->GetStateCost(0);
        std::cout << "Cost: " << cost << std::endl;
    }

    void RunAlgorithm(const double& time_limit) {
        while (t < time_limit) {
            Algorithm();
            r.sleep();
            t += dt;    
        }

        if (isTraj == true) {
                std::shared_ptr<Trajectory> trajectory = std::make_shared<Trajectory>(DefineTrajectory());
                scene->AddTrajectory("TargetRelative",trajectory);
                t = 0.0;
                std::cout << "Performing Trajectory" << std::endl;
                while (t < time_limit) {
                    Algorithm();
                    r.sleep();
                    t += dt;    
                }
                scene->RemoveTrajectory("TargetRelative");
            }

    }

    //Returns the average time (in microseconds) to run each iteration of the algorithm
    double GetAverageTime() {
        size_t n = t_algorithm.size();
        std::chrono::microseconds sum(0);
        for (size_t i = 0; i < n; i++) {
            sum += t_algorithm[i];
        }
        return static_cast<double>(sum.count()) / n; //Returns the average time to run each iteartion of the algorithm
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
    bool isTraj; //If a trajectory algorithm is being tested
    std::unique_ptr<Eigen::MatrixXd> prevU; //Matrix to store the previous solution
    std::vector<std::chrono::microseconds> t_algorithm; //Vector that stores time of each iteration of the algorithm 

};

