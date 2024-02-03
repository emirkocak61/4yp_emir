#pragma once

#include <ros/ros.h>
#include <exotica_core/exotica_core.h>
#include <unitree_arm_sdk/control/unitreeArm.h>

class LowPassFilter {
private:
    double alpha;
    Eigen::VectorXd filtered_velocity;

public:
    LowPassFilter(double alpha)
        : alpha(alpha) {
            //Initialize with zeros
            filtered_velocity = Eigen::VectorXd::Zero(6);
        }

    Eigen::VectorXd filter(Eigen::VectorXd& velocity) {
        // Apply the low-pass filter formula
        filtered_velocity = alpha * velocity + (1 - alpha) * filtered_velocity;
        return filtered_velocity;
    }
};

class ArmControl {
public: 
    ArmControl() : arm(true), velocity_filter(0.167) {
        //Start communications with the arm
        arm.sendRecvThread->start();
        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
        std::vector<double> KP = {200,300,300,200,150,100};
        std::vector<double> KD = {1000,1000,1000,1000,1000,1000};
        arm.lowcmd->setControlGain(KP, KD);
        t = 0.0;
        dt = 0.02;
    }
    ~ArmControl() {solver.reset();}
    
    void Run() {
        UNITREE_ARM::Timer timer(dt); //Sets the control frequency to 50Hz
        while(t < 10) {
            //X = getRobotState();
            Eigen::VectorXd state = getRobotState();
            Eigen::VectorXd state_error = X - state;
            std::cout << "Mean State error =  " << state_error.norm() << std::endl;
            problem->SetStartTime(t);
            problem->SetStartState(state);
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
            sendArmCommand(X.head(6));
            t += dt;
        }
    }

    void sendArmCommand(const Eigen::VectorXd& targetQ) {
        arm.sendRecvThread->shutdown();
        Vec6 initQ = arm.lowstate->getQ();
        double duration = 10; 
        UNITREE_ARM::Timer timer2(0.002); //This is the default control frequency of 500Hz
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
            timer2.sleep();
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

    //Sets up EXOTica
    void SetupPlanningProblem(const std::string& config_path) {
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
        Eigen::VectorXd arm_state = arm.lowstate->getQ();
        
        //Set start state in EXOTica
        scene->SetModelState(arm_state);
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
    UNITREE_ARM::unitreeArm arm;
    Eigen::VectorXd X; //vector to store robot state
    //EXOTica related objects
    exotica::MotionSolverPtr solver; //Solver for the optimization problem
    exotica::DynamicTimeIndexedShootingProblemPtr problem; //EXOTica planning problem
    exotica::ScenePtr scene;
    std::unique_ptr<Eigen::MatrixXd> prevU; //Matrix to store previous control inputs

    double t; //Problem time
    double dt; //Control time-step

    LowPassFilter velocity_filter;
};

