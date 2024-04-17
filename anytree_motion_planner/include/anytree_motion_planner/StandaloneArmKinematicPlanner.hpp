#pragma once

#include <exotica_core/exotica_core.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

using namespace exotica;

class StandaloneArmKinematicPlanner {
public:
    StandaloneArmKinematicPlanner(const std::string& actionName) : action_name(actionName),dt(0.02), t(0.0), rate(50) {
        q = Eigen::VectorXd::Zero(arm_dof);
        qd = Eigen::VectorXd::Zero(arm_dof);
        arm_state = Eigen::VectorXd::Zero(arm_dof);
        relative_joint_limits = Eigen::MatrixXd::Zero(arm_dof, 2);
        //Initialize ros publisher and subscribers
        mp_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan",10);
        arm_state_subsciber = nh_.subscribe("/z1_gazebo/joint_states_filtered",10,&StandaloneArmKinematicPlanner::ArmStateCb,this);
        ros::Duration(1.0).sleep();
        SetupProblem();
    }

    ~StandaloneArmKinematicPlanner() {
        this->solver.reset();
        Setup::Destroy();}
    
    void SetupProblem() {
        Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(action_name)));
        solver = XMLLoader::LoadSolver("{anytree_motion_planner}/resources/configs/kinematic/" + action_name + "StandaloneArm_kinematic.xml");
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);
        //Get EXOTica problem
        problem = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(solver->GetProblem());
        //Get EXOTica scene
        scene = problem->GetScene();
        scene->SetModelState(arm_state);
        problem->Update(scene->GetControlledState(),0);
        relative_joint_limits = scene->GetKinematicTree().GetJointLimits();
        std::cout << "Joint limits loaded" << std::endl;
    }


    virtual void SetupGoal(const KDL::Frame target_frame) {
        scene->AttachObjectLocal("Target","", target_frame);
        int T = problem->GetT();
        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        double alpha = 0.99;
        double rho = 1e1;
        for (int t(0); t < T; t++) {
            //rho = pow(alpha,t) * 1e3; //Rho decreses with each time step
            problem->cost.SetGoal("Position", goal, t);
            problem->cost.SetRho("Position",rho,t);
        }
        problem->cost.SetRho("Position",1e3,T-1);
    }



    void Iterate() {
        problem->SetStartTime(t);
        problem->SetStartState(arm_state);
        //Create a solution container
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        //Solve problem
        solver->Solve(*solution);
        //Calculate velocities
        Eigen::VectorXd qnew(arm_dof);
        qnew = solution->row(1);
        qd = (qnew-q)/(dt);
        q = qnew;
        //Update problem
        problem->Update(q.transpose(),1);
        scene->GetKinematicTree().PublishFrames(); 

    }

    void PublishToRobot() {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.points.reserve(1);
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(arm_dof);
        trajectory_point.velocities.resize(arm_dof); 
        //Convert from Eigen::VectorXd to std::vector
        // Use Eigen::Map to directly map trajectory_point.positions to an Eigen::VectorXd
        Eigen::Map<Eigen::VectorXd>(trajectory_point.positions.data(), trajectory_point.positions.size()) = q;
        Eigen::Map<Eigen::VectorXd>(trajectory_point.velocities.data(),trajectory_point.velocities.size()) = qd;
        trajectory_msg.points.push_back(trajectory_point);
        mp_publisher.publish(trajectory_msg);  
    }

    double GetError() {
        Eigen::VectorXd error = problem->cost.GetTaskError("Position",0);
        return error.norm();
    }

    void ArmStateCb(const sensor_msgs::JointStateConstPtr &state) {
        std::lock_guard<std::mutex> lock(arm_mutex);
        Eigen::Map<const Eigen::VectorXd> tempMap(state->position.data(), arm_dof);
        arm_state = tempMap;
    }


    KDL::Frame GetFrameFromPose(const geometry_msgs::Pose &pose) {
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

    virtual void PerformMotion() {};
    virtual void PerformTrajectory(const std::shared_ptr<Trajectory>& trajectory) {};
protected:
    std::string action_name;
    int arm_dof = 6;
    //Problem related data
    double dt; //Problem time step
    double t; //Variable to keep track the problem time
    double counter_limit = 10;
    double time_limit = 50.0; //Time limit for the action
    double joint_limit_tolerance = 0.0001;
    double tolerance_ = 7.5e-2;
    double t_limit = 50.0;
    Eigen::VectorXd q; //Vector to store joint positions
    Eigen::VectorXd qd; //Vector to store joint velocities
    Eigen::VectorXd arm_state;
    Eigen::MatrixXd relative_joint_limits;
    //Mutexes to protect data
    std::mutex arm_mutex;
    std::mutex base_mutex; //Mutex to protect base pose data

    //ROS related objects
    ros::NodeHandle nh_;
    ros::V_string joint_names;
    ros::Rate rate;
    ros::Publisher mp_publisher;
    ros::Subscriber arm_state_subsciber;
    //EXOTica Related Objects
    MotionSolverPtr solver;
    UnconstrainedTimeIndexedProblemPtr problem;
    ScenePtr scene;
};

