/*
    Script to test the AICO solver for planning a trajectory for motion planning
*/
#pragma once

#include <exotica_core/exotica_core.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace exotica;

class TestAICO {
    typedef KDL::Frame Frame;
public:

    TestAICO() : dt(0.02), r(1/dt) {}
    ~TestAICO() {}

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
        solver->debug_ = false;
        solver->SetNumberOfMaxIterations(1);      
        t = 0.0;
        std::cout << "Problem all set up" << std::endl;
        
    }

    void SetupGoal(const Frame target_frame) {
        scene->AttachObjectLocal("Target","", target_frame);
        int T = problem->GetT();
        std::cout << "Problem T: " << T << std::endl;
        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        for (int t(0); t < T; t++) {
            problem->SetGoal("Position",goal,t);
            problem->SetRho("Position",1e1,t);
        }
        problem->SetGoal("Position",goal,-1);
        problem->SetRho("Position", 1e4,-1);
        std::cout << "Problem goal set" << std::endl;
    }

    void Algorithm() {
        problem->SetStartTime(0);
        problem->SetStartState(q);
        //Create a solution container
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        //Solve problem
        auto start = std::chrono::high_resolution_clock::now();
        solver->Solve(*solution);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
        t_algorithm.push_back(duration);
        q = solution->row(1);
        std::cout << "q: " << q.transpose() << std::endl;
        //Update problem
        problem->Update(q.transpose(),0);
        std::cout << "Model State:" << scene->GetModelState().transpose() << std::endl;
        scene->GetKinematicTree().PublishFrames();        
    }

    void PublishTrajectoryToRVIZ(const std::unique_ptr<Eigen::MatrixXd>& solution) {
        ros::Rate loop_rate(50.0);
        int t = 0;
        int PlaybackWaitInterval = 30;
        while (ros::ok())
        {
            int i = 1;
            if (t == 0 || t == solution->rows() - 1) i = PlaybackWaitInterval;
            while (i-- > 0)
            {
                problem->Update(solution->row(t).transpose(),t);
                std::cout << "Model State:" << scene->GetModelState().transpose() << std::endl;
                scene->GetKinematicTree().PublishFrames();
                ros::spinOnce();
                loop_rate.sleep();
            }
            t = t + 1 >= solution->rows() ? 0 : t + 1;
        }
        
    }

    double GetAverageTime() {
        size_t n = t_algorithm.size();
        std::chrono::microseconds sum(0);
        for (size_t i = 0; i < n; i++) {
            sum += t_algorithm[i];
        }
        return static_cast<double>(sum.count()) / n; //Returns the average time to run each iteartion of the algorithm
    }

    void Run() {
        t=0;
        int i = 0;
        while (t < time_limit) {
            std::cout << "Iteration: " << i << std::endl;
            Algorithm();
            ros::spinOnce();
            r.sleep();
            t += dt;
            std::cout << "t: " << t << std::endl;
            i += 1;
        }
    }


protected:
    int arm_dof = 6;
    double time_limit = 10.0;
    //EXOTica related objects
    MotionSolverPtr solver;
    UnconstrainedTimeIndexedProblemPtr problem;
    ScenePtr scene;
    Eigen::VectorXd q; //Vector to store joint positions

    ros::NodeHandle nh_;
    ros::V_string joint_names;
    ros::Rate r;

    double dt; //Problem time step
    double t; //Variable to keep track the problem time
    std::vector<std::chrono::microseconds> t_algorithm; //Vector that stores time of each iteration of the algorithm 
};