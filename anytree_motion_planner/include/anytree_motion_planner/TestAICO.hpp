/*
    Script to test the AICO solver for planning a trajectory for motion planning
*/
#pragma once

#include <exotica_core/exotica_core.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>

using namespace exotica;

class TestAICO {
    typedef KDL::Frame Frame;
public:

    TestAICO() : r(20) {
        dataFile = std::ofstream("position_data.csv"); //File to record position data
        dt = 0.05;
        mp_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan",10);
        state_subsciber = nh_.subscribe("/z1_joint_states",10,&TestAICO::RobotStateCb,this);
    }
    ~TestAICO() {dataFile.close();}

    void SetupProblem(const std::string& config_path,
                      const Eigen::VectorXd& start_state) {

        Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("test_node")));
        solver = XMLLoader::LoadSolver(config_path);
        //Get Problem
        PlanningProblemPtr planning_problem = solver->GetProblem(); //This returns a generic problem
        //Downcast the pointer to a UnconstrainedTimeIndexedProblem pointer
        problem = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(planning_problem);
        //Get exotica scene
        scene = problem->GetScene();
        scene->SetModelState(start_state);
        problem->Update(scene->GetControlledState(),0);
        scene->GetKinematicTree().PublishFrames();
        //Initialize q
        q = Eigen::VectorXd::Zero(arm_dof);
        qd = Eigen::VectorXd::Zero(arm_dof);
        robot_state = Eigen::VectorXd::Zero(arm_dof);
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);      
        t = 0.0;
        std::cout << "Problem all set up" << std::endl;   
    }

    void PublishMotionPlan() {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.points.reserve(1);
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(6);
        trajectory_point.velocities.resize(6); 
        //Convert from Eigen::VectorXd to std::vector
        // Use Eigen::Map to directly map trajectory_point.positions to an Eigen::VectorXd
        Eigen::Map<Eigen::VectorXd>(trajectory_point.positions.data(), trajectory_point.positions.size()) = q;
        Eigen::Map<Eigen::VectorXd>(trajectory_point.velocities.data(),trajectory_point.velocities.size()) = qd;
        trajectory_msg.points.push_back(trajectory_point);
        mp_publisher.publish(trajectory_msg);  
    }

    void RobotStateCb(const sensor_msgs::JointStateConstPtr &state) {
        std::lock_guard<std::mutex> lock(state_mutex);
        Eigen::Map<const Eigen::VectorXd> tempMap(state->position.data(), state->position.size());
        robot_state = tempMap;
    }

    void SetupGoal(const Frame target_frame) {
        scene->AttachObjectLocal("Target","", target_frame);
        int T = problem->GetT();
        std::cout << "Problem T: " << T << std::endl;
        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        for (int t(0); t < T; t++) {
            problem->SetGoal("Position",goal,t);
            problem->SetRho("Position",1e2,t);
        }
        problem->SetGoal("Position",goal,-1);
        problem->SetRho("Position", 1e6,-1);
        std::cout << "Problem goal set" << std::endl;
    }

    void Algorithm() {
        problem->SetStartTime(0);
        problem->SetStartState(robot_state);
        //Create a solution container
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        //Solve problem
        solver->Solve(*solution);
        //Calculate velocities
        Eigen::VectorXd qnew(6);
        qnew = solution->row(1);
        qd = (qnew-q)/(dt);
        q = qnew;
        std::cout << "q: " << q.transpose() << std::endl;
        //Update problem
        problem->Update(q.transpose(),0);
        scene->GetKinematicTree().PublishFrames();        
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
            ros::spinOnce();
            r.sleep();
            t += dt;
            std::cout << "t: " << t << std::endl;
            i += 1;
        }
    }


protected:
    int arm_dof = 6;
    double time_limit = 50.0;
    //EXOTica related objects
    MotionSolverPtr solver;
    UnconstrainedTimeIndexedProblemPtr problem;
    ScenePtr scene;
    Eigen::VectorXd q; //Vector to store joint positions
    Eigen::VectorXd qd; //Vector to store joint velocities
    std::mutex state_mutex;
    Eigen::VectorXd robot_state;

    ros::NodeHandle nh_;
    ros::V_string joint_names;
    ros::Rate r;
    ros::Publisher mp_publisher;
    ros::Subscriber state_subsciber;

    double dt; //Problem time step
    double t; //Variable to keep track the problem time
    std::vector<Eigen::VectorXd> position_data; //Vector to stor the position data at each time step
    std::vector<double> time_step; //Vector to store time step information
    std::ofstream dataFile;

};