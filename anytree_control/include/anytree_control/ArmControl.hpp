#pragma once

#include <ros/ros.h>
#include <exotica_core/exotica_core.h>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <anytree_control/LowPassFilter.hpp>

class ArmControl {
public: 
    ArmControl() : r(50), robot_state(Eigen::VectorXd::Zero(12)) {
        //Constructor for the unitree arm
        //arm(true),
        //Initialize the velocity filter
        //velocity_filter(0.167,Eigen::VectorXd::Zero(6)) {
        //SetupArm();
        motion_plan_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan",10);
        state_sub = nh_.subscribe("/z1_joint_states", 10, &ArmControl::stateCallback, this);
        t = 0.0;
        dt = 0.02;
    }
    ~ArmControl() {solver.reset();}
    
    void Run() {
        ; //Sets the control frequency to 50Hz
        while(t < 10) {
            //X = getRobotState();
            //Eigen::VectorXd state = getRobotState();
            //std::cout << "Robot state: " << robot_state.transpose() << std::endl;
            Eigen::VectorXd error = X - robot_state;
            std::cout << "State error norm: " << error.norm() << std::endl;
            state_error.push_back(error.norm());
            problem->SetStartTime(t);
            problem->SetStartState(robot_state);
            //Set the previous solution as initial guess
            problem->set_U(prevU->transpose());

            //Initialize solution container
            std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
            solver->Solve(*solution);
            //Apply only the first step of the solution
            problem->Update(X,solution->row(0),0);
            //Swap prevU and solution pointers for next iteration
            prevU.swap(solution);

            //Update joint states
            X = problem->get_X(1);
            scene->GetKinematicTree().PublishFrames();
            PublishMotionPlan();
            //sendArmCommand(X.head(6));
             //Wait for publish motion plan to finish
            t += dt;
            r.sleep();
        }
        double mean_error = 0;
        for (size_t i = 0; i < state_error.size(); i++) {mean_error += state_error[i];}
        mean_error = mean_error/(static_cast<double>(state_error.size()));
        std::cout << "Mean state error: " << mean_error << std::endl;
    }
    void PublishMotionPlan() {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.points.reserve(1);
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(6);
        trajectory_point.velocities.resize(6); 
        trajectory_point.accelerations.resize(6);//Ensure the vector has 6 elements
        //Convert from Eigen::VectorXd to std::vector
        // Use Eigen::Map to directly map trajectory_point.positions to an Eigen::VectorXd
        Eigen::Map<Eigen::VectorXd>(trajectory_point.positions.data(), trajectory_point.positions.size()) = X.head(6);
        Eigen::Map<Eigen::VectorXd>(trajectory_point.velocities.data(),trajectory_point.velocities.size()) = X.tail(6);
        //Eigen::Map<Eigen::VectorXd>(trajectory_point.accelerations.data(),trajectory_point.accelerations.size()) = prevU->row(1);
        trajectory_msg.points.push_back(trajectory_point);
        motion_plan_publisher.publish(trajectory_msg);   
    }

    void stateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        Eigen::Map<Eigen::VectorXd>(robot_state.data(), 6) = Eigen::VectorXd::Map(&msg->position[0], 6);
        Eigen::Map<Eigen::VectorXd>(robot_state.data() + 6, 6) = Eigen::VectorXd::Map(&msg->velocity[0], 6);
    }
    /*
    void updateRobotState() {
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/z1_joint_states");
        Eigen::Map<const Eigen::VectorXd> joint_position(joint_states->position.data(), 6);
        Eigen::Map<const Eigen::VectorXd> joint_velocities(joint_states->velocity.data(), 6);

        // Preallocate the size of the state vector to avoid dynamic memory allocations
        Eigen::VectorXd state(12);
        
        // Concatenate position and velocity vectors efficiently
        state.head(6) = joint_position;
        state.tail(6) = joint_velocities;

        robot_state = std::move(state);  // Move assignment to avoid copy
    }
    */
    /*
    void sendArmCommand(const Eigen::VectorXd& targetQ) {
        arm.sendRecvThread->shutdown();
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
    
    
    Eigen::VectorXd getRobotState() {
        Eigen::VectorXd robot_state(12);
        Eigen::VectorXd arm_position = arm.lowstate->getQ();
        Eigen::VectorXd arm_velocity = arm.lowstate->getQd();
        Eigen::VectorXd filtered_v = velocity_filter.filter(arm_velocity);
        robot_state << arm_position, filtered_v;
        return robot_state;
    }
    
    void SetupArm() {
        //Start communications with the arm
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        std::vector<double> KP = {200,300,300,200,150,100};
        std::vector<double> KD = {1000,1000,1000,1000,1000,1000};
        arm.lowcmd->setControlGain(KP, KD);
    }
    */

    //Sets up EXOTica
    void SetupPlanningProblem(const std::string& config_path,const Eigen::VectorXd& start_state) {
        //Setup EXOTica ROS
        exotica::Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("test_node")));

        solver = exotica::XMLLoader::LoadSolver(config_path);
        solver->debug_= false;
        solver->SetNumberOfMaxIterations(1);
        exotica::PlanningProblemPtr p = solver->GetProblem();
        //Static cast the problem to the specific problem instance
        problem = std::static_pointer_cast<exotica::DynamicTimeIndexedShootingProblem>(p);
        //Get EXOTica scene
        scene = problem->GetScene();
        //Get arm state
        //Eigen::VectorXd arm_state = arm.lowstate->getQ();
        
        //Set start state in EXOTica
        scene->SetModelState(start_state);
        problem->Update(scene->GetControlledState(),0);
        //Initialize the prevU object
        int T = problem->get_T();
        int n = scene->get_num_controls();
        int nv = scene->get_num_velocities();
        std::cout << "Number of controlled joints: " << n << std::endl;
        std::cout << "Number of velocities " << nv << std::endl;
        prevU = std::make_unique<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(T-1,n));
        //Initialize state vector
        X = problem->GetStartState();  
    }

    void SetGoal(const KDL::Frame &target_frame) {
        scene->AttachObjectLocal("Target","", target_frame);
        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        int T = problem->get_T();
        for (int t = 0; t < T; ++t) {
            problem->cost.SetGoal("Position", goal, t);
            problem->cost.SetRho("Position",1e1,t);
            }
        problem->cost.SetGoal("Position", goal,T-1);
        problem->cost.SetRho("Position",1e4,T-1);

    }
protected:
    ros::NodeHandle nh_;
    ros::Publisher motion_plan_publisher;
    ros::Rate r;
    //UNITREE_ARM::unitreeArm arm;
    Eigen::VectorXd X; //vector to store robot state
    //EXOTica related objects
    exotica::MotionSolverPtr solver; //Solver for the optimization problem
    exotica::DynamicTimeIndexedShootingProblemPtr problem; //EXOTica planning problem
    exotica::ScenePtr scene;
    std::unique_ptr<Eigen::MatrixXd> prevU; //Matrix to store previous control inputs
    Eigen::VectorXd robot_state;
    std::mutex state_mutex;
    ros::Subscriber state_sub;

    double t; //Problem time
    double dt; //Control time-step
    //LowPassFilter<Eigen::VectorXd> velocity_filter;
    std::vector<double> state_error;
};

