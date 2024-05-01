//The c++ implementation of the motion planner base class written by Jacques
//This will allow faster runtime and easier use for further user as the EXOTica docs are more clear in c++
//It is important to check where vectors are std::vector or Eigen::Matrix

#include "anytree_motion_planner/MotionPlannerStandaloneArmBaseClass.hpp"
#include <cmath>

using namespace exotica;


    //Base class for an MPC and EXOTica based motion planner, implemented as a ros action server
    //Abstract base class, must implement:
    //Action specific intialization, defining and starting action server
    //Feedback and result action variables
    //Callback for the action server
    //Also requires a main function OUTSIDE class definition

    //Class constructor
    MotionPlannerStandaloneArmBaseClass::MotionPlannerStandaloneArmBaseClass(const std::string& actionName) : 
                                                                            action_name(actionName), dt(0.02), rate(50),
                                                                            start_tolerance(15.0e-2),
                                                                            gradient_tolerance(1.0e-4),
                                                                            gradient_counter_limit(10),
                                                                            convergence_tolerance(5.0e-2),
                                                                            previous_error(0.0),
                                                                            error_metric("Position"),tolerance_(1.0e-2),
                                                                            counter_limit(10), t_limit(90.0), self_tolerance(5e-2),
                                                                            self_counter(10), listener(tfBuffer)
        {   
            //Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("as")));
            motion_plan_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan", 10);
            error_publisher = nh_.advertise<std_msgs::Float64>("/motion_plan_error", 10);
            state_subscriber = nh_.subscribe("/z1_gazebo/joint_states_filtered",10,&MotionPlannerStandaloneArmBaseClass::RobotStateCb,this);
            std::cout << "Action name: " << action_name << std::endl;
            solver = XMLLoader::LoadSolver("{anytree_motion_planner}/resources/configs/dynamic/" + action_name + "StandaloneArm_dynamic.xml");
            PlanningProblemPtr planning_problem =  solver->GetProblem();
            problem = std::dynamic_pointer_cast<DynamicTimeIndexedShootingProblem>(planning_problem);

            //Check if the dynamic_pointer cast failed
            if (!problem) {
                std::cerr << "Dynamic pointer cast failed" << std::endl;
                ros::shutdown();
                exit(1);
            }
            else {
                scene = problem->GetScene();
                joint_names = scene->GetControlledJointNames();
                 
                kinematic_tree = &(scene->GetKinematicTree());
                Eigen::MatrixXd joint_limits = kinematic_tree->GetJointLimits();
                joint_limit_tolerance = 0.0001;
    
                joint_velocities = kinematic_tree->GetVelocityLimits();
                        
                int T = problem->get_T();
                Eigen::VectorXd goal(6);
                Eigen::VectorXd joint_limit  =  Eigen::VectorXd::Zero(scene->get_num_controls()*2);
                goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                Eigen::VectorXdRefConst goal_ref = goal;
                double alpha = 0.8;
                double rho;
                for (int t = 0; t < T; ++t) {
                    problem->cost.SetGoal("Position", goal_ref, t);
                    problem->cost.SetGoal("JointLimit", joint_limit,t);
                    problem->cost.SetRho("Position",1e1,t);
                    problem->cost.SetRho("JointLimit",1e3,t);
                }
                problem->cost.SetGoal("Position", goal_ref,T-1);
                problem->cost.SetRho("Position",1e3,T-1);
        
                solver->debug_ = false;
                solver->SetNumberOfMaxIterations(1);

                prevU = std::make_unique<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(T-1,6));
                robot_state = Eigen::VectorXd::Zero(12);
                q = robot_state;
                scene->GetKinematicTree().PublishFrames();
                
                //std::string filePath = "/home/emirkocak/Documents/position_comparison.txt";
                //outFile = std::ofstream(filePath.c_str());  
            }
        }

    MotionPlannerStandaloneArmBaseClass::~MotionPlannerStandaloneArmBaseClass(){
        //outFile.close();
        solver.reset();
        Setup::Destroy();   //Destroy all the exotica related objects
    }

    //Initializes the robot pose in EXOTica
    void MotionPlannerStandaloneArmBaseClass::InitRobotPose() {
        q = Eigen::VectorXd(problem->GetStartState()); //Ensures that q is a copy of the returned vector
        if ((joint_names.size()) != 0 ) {
            //Wait for a joint state message from the arm and call the callback function
            boost::shared_ptr<const sensor_msgs::JointState> joint_state_msg_ptr;
            joint_state_msg_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/z1_gazebo/joint_states");
            JointStatesCb(*joint_state_msg_ptr);
        }
    }
    //Publish motion plan as a single waypoint trajectory_msgs/JointTrajectory
    void MotionPlannerStandaloneArmBaseClass::PublishToRobot() {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.joint_names = joint_names;
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(6); //Ensure the vector has 6 elements
        trajectory_point.velocities.resize(6); //Ensure the vector has 6 elements
        //Convert from Eigen::VectorXd to std::vector
        Eigen::Map<Eigen::VectorXd>(trajectory_point.positions.data(), trajectory_point.positions.size()) = q.head(6);
        Eigen::Map<Eigen::VectorXd>(trajectory_point.velocities.data(), trajectory_point.velocities.size()) = q.tail(6);
        ros::Duration duration;
        trajectory_point.time_from_start = duration.fromSec(dt);
        trajectory_msg.points.push_back(trajectory_point);
        motion_plan_publisher.publish(trajectory_msg);
    }

    double MotionPlannerStandaloneArmBaseClass::GetError() {
        //A weird implementation going on here...
        std_msgs::Float64 error_msg;
        double state_cost = problem->GetStateCost(0);
        double square_error = state_cost * state_cost;
        error_msg.data = std::sqrt(square_error);
        error_publisher.publish(error_msg);
        return std::sqrt(square_error);

    }

    void MotionPlannerStandaloneArmBaseClass::JointStatesCb(const sensor_msgs::JointState &joint_states){
        //Assumes all joint_states messages contain all joints
        //Iterate over the joint names defined for the robot.
        for (size_t i=0;i < joint_names.size(); i++){
            //Search for the current joint name in the received joint_states message.
            //std::find returns an iterator to the found element or to the end of the container if not found. 
            auto it = std::find(joint_states.name.begin(), joint_states.name.end(), joint_names[i]);

            //Check if the joint name is found in the joint_states message
            if (it != joint_states.name.end()) {
                //Calculate the index of the found joint name.
                //std::distance calculates tthe number of elements between two iterators
                int index = std::distance(joint_states.name.begin(), it);
                //Update the corresponding joint position in q.
                //The index is used to access the correct position value in the joint_states message
                q(i) = joint_states.position[index];
            }
        }
    }

    void MotionPlannerStandaloneArmBaseClass::Iterate() {
        problem->SetStartTime(t); //Set problem start time
        problem->SetStartState(q); //Set problem start state
        //Set the previous solution as initial guess
        problem->set_U(prevU->transpose());
        //Initialize solution container
        std::unique_ptr<Eigen::MatrixXd> solution = std::make_unique<Eigen::MatrixXd>();
        solver->Solve(*solution);
        
        //Apply only the first step of the solution
        problem->Update(q,solution->row(0),0);
        //Swap prevU and solution pointers for next iteration
        prevU.swap(solution);
        //Update Joint Positions
        q = problem->get_X(1);
        scene->GetKinematicTree().PublishFrames();
        scene->SetModelState(q.head(scene->get_num_positions()));
    }

    
    KDL::Frame MotionPlannerStandaloneArmBaseClass::GetFrameFromPose(const geometry_msgs::Pose &pose) {
        
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

    void MotionPlannerStandaloneArmBaseClass::RobotStateCb(const sensor_msgs::JointStateConstPtr &state) {
        std::lock_guard<std::mutex> lock(state_mutex);
        Eigen::Map<const Eigen::VectorXd> positionMap(state->position.data(), 6);
        Eigen::Map<const Eigen::VectorXd> velocityMap(state->position.data(),6);
        robot_state << positionMap , velocityMap;    
    }
        




