#pragma once

#include <exotica_core/exotica_core.h>
#include <exotica_core/trajectory.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <bt_drs_msgs/approachTargetAction.h>
#include <bt_drs_msgs/approachTargetFeedback.h>
#include <bt_drs_msgs/approachTargetResult.h>
#include <anytree_control/RobotInterface.hpp>


using namespace exotica;

class MPCMotionPlanner {
public:
    MPCMotionPlanner() : t(0.0), dt(0.02), r(50), robot(0.02), 
        //Setup the ROS Action Servers
        as_(nh_,"approachTarget_as", boost::bind(&MPCMotionPlanner::approachTargetCb,this,_1),false) {
        as_.start();
        //Setup EXOTica
        solver = XMLLoader::LoadSolver("{anytree_motion_planner}/resources/configs/approachTarget.xml");
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);
        PlanningProblemPtr planning_problem = solver->GetProblem();
        //Downcast the plannin_problem into a DynamicTimeIndexed problem
        problem = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(planning_problem);
        scene = problem->GetScene();
        //Sets the global joint limits
        SetJointLimits();
        ros::V_string joint_names= scene->GetControlledJointNames();
        int T = problem->get_T();
        int n = scene->get_num_controls();
        prevU = std::make_unique<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(T-1,n));
        //Initialize motion plan vector
        SetGoal();
        X = problem->GetStartState();
    }
    void approachTargetCb(const bt_drs_msgs::approachTargetGoalConstPtr &goal) {
        //Define target frame
        KDL::Frame T_approach;
        T_approach = GetFrameFromPose(goal->target);
        //Attach target in the absolute world frame
        scene->AttachObjectLocal("Target","",T_approach);
        ROS_INFO("Approaching Target");
        result_.result = true;
        double t_limit = 5;
        t = 0.0;
        int counter = 0;
        int counter_limit = 20;
        double tolerance_ = 2.5e-2;

        while (counter < counter_limit && t < t_limit){
            if (as_.isPreemptRequested()) {
                result_.result = false;
                ROS_WARN("Approach Target: PREEMPTED (Goal Cancelled)");
                as_.setPreempted();
                break;
            }

            Iterate(); //Generate motion plan via EXOTica
            double error = GetError(); //Calculate error
            ROS_INFO("Current Error: %f", error);

            //Publish feedback to client
            feedback_.error = error;
            as_.publishFeedback(feedback_); 

            //Check whether to increment the counter
            if (error < tolerance_) {counter++;}

            //Sleep and increment time
            r.sleep();
            t = t + dt;  
        }

        //Check that EEF has reached target
        if (result_.result == true) {
            double error = GetError();
            if (error < tolerance_){
                ROS_INFO(" SUCCESS (Approached Target)");
                as_.setSucceeded(result_);
            }
            else {
                result_.result = false;
                ROS_WARN("ABORTED (Did not reached target within time limit)");
                as_.setAborted(result_);
            }
        }
        problem->SetStartTime(0.0); //Reset problem start time
    }
    void Iterate() {
        SetJointLimits();
        Eigen::VectorXd state = robot.getRobotState();
        Eigen::VectorXd error = X - state;
        std::cout << "State error norm: " << error.norm() << std::endl;
    
        problem->SetStartTime(t);
        problem->SetStartState(X);
        //Set the previous solution as initial guess
        //problem->set_U(prevU->transpose());

        //Initialize solution container
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        solver->Solve(*solution);
        //Apply only the first step of the solution
        problem->Update(X,solution->row(0),0);
        //Swap prevU and solution pointers for next iteration
        //prevU.swap(solution);
        //Update joint states
        X = problem->get_X(1);
        scene->SetModelState(X.head(scene->get_num_positions()));
        //robot.publishMotionPlan(X); //Send motion plan to the robot
    }

    void SetGoal() {
        int T = problem->get_T();
        Eigen::VectorXd goal(6);
        Eigen::VectorXd joint_limit  =  Eigen::VectorXd::Zero(scene->get_num_controls()*2);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::VectorXdRefConst goal_ref = goal;
        for (int t = 0; t < T; ++t) {
            problem->cost.SetGoal("Position", goal_ref, t);
            problem->cost.SetGoal("JointLimit", joint_limit,t);
            problem->cost.SetRho("Position",1e1,t);
            problem->cost.SetRho("JointLimit",1e3,t);
        }
        problem->cost.SetGoal("Position", goal_ref,T-1);
        problem->cost.SetRho("Position",1e3,T-1);
    }
    double GetError() {
        return problem->GetStateCost(0);    
    }
    void SetJointLimits() {
        Eigen::VectorXd state = robot.getRobotState();
        double tolerance = 0.001;
        std::map<std::string,double> model_state;
        model_state["world_joint/trans_x"] = state(0);
        model_state["world_joint/trans_y"] = state(1);
        model_state["world_joint/trans_z"] = state(2);
        model_state["world_joint/rot_x"] = state(3);
        model_state["world_joint/rot_y"] = state(4);
        model_state["world_joint/rot_z"] = state(5);
        scene->SetModelState(model_state); //Sets the state of the base in exotica
        
        Eigen::VectorXd upper_limits(robot.base_dof), lower_limits(robot.robot_dof);
        upper_limits = scene->GetKinematicTree().GetJointLimits().col(1);
        lower_limits = scene->GetKinematicTree().GetJointLimits().col(0);
        
        
        for (int i = 0; i < robot.base_dof; i++) {
            upper_limits(i) += state(i) + tolerance;
            lower_limits(i) += state(i) - tolerance;
        }

        std::cout << "Upper: " << upper_limits << std::endl;
        std::cout << "Lower: " << lower_limits << std::endl;

        scene->GetKinematicTree().SetJointLimitsUpper(upper_limits);
        scene->GetKinematicTree().SetJointLimitsLower(lower_limits);
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

private:
    ros::NodeHandle nh_;
    std::string robot_name;
    double t; //Problem time
    double dt; //Control frequency of the robot
    ros::Rate r; 

    RobotInterface robot; //Object that handles communications with the robot

    //EXOTica related objects
    MotionSolverPtr solver; //EXOTica solver object
    DynamicTimeIndexedShootingProblemPtr problem; //EXOTica planning problem object
    ScenePtr scene; //EXOTica planning scene object
    std::unique_ptr<Eigen::MatrixXd> prevU; //Matrix to store previous control inputs
    Eigen::VectorXd X; //Vector to store robot_state in exotica model 
    
    actionlib::SimpleActionServer<bt_drs_msgs::approachTargetAction> as_;
    bt_drs_msgs::approachTargetFeedback feedback_;
    bt_drs_msgs::approachTargetResult result_;
    //Testing related objects
    //std::vector<double> state_error;


    
};