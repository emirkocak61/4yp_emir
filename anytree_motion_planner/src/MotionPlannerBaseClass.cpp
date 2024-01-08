//The c++ implementation of the motion planner base class written by Jacques
//This will allow faster runtime and easier use for further user as the EXOTica docs are more clear in c++
//It is important to check where vectors are std::vector or Eigen::Matrix

#include "exotica_core/exotica_core.h" //Include exotica
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_listener.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <array>
#include <vector>
#include <chrono>
#include <cmath>
#include "anytree_motion_planner/MotionPlannerBaseClass.hpp"


using namespace exotica;


    //Base class for an MPC and EXOTica based motion planner, implemented as a ros action server
    //Abstract base class, must implement:
    //Action specific intialization, defining and starting action server
    //Feedback and result action variables
    //Callback for the action server
    //Also requires a main function OUTSIDE class definition

    //Class constructor
    MotionPlannerBaseClass::MotionPlannerBaseClass(const std::string& actionName) : 
                                action_name(actionName), dt(0.02), rate(1/dt),
                                base_dof(6),start_tolerance(5e-2),
                                error_metric("Position"),tolerance_(2.5e-2),
                                counter_limit(10), t_limit(90.0), self_tolerance(5e-2),
                                self_counter(10), listener(tfBuffer)
        {
            motion_plan_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/motion_plan", 10);
            std::cout << "Action name: " << action_name << std::endl;
            solver = XMLLoader::LoadSolver("{anytree_motion_planner}/resources/configs/" + action_name + ".xml");
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
                rest_pose = LookupBasePose();
                 
                kinematic_tree = &(scene->GetKinematicTree());
                Eigen::MatrixXd joint_limits = kinematic_tree->GetJointLimits();
                relative_joint_limits_lower = joint_limits.col(0);
                relative_joint_limits_upper = joint_limits.col(1);
                joint_limit_tolerance = 0.0001;
                SetJointLimits();
    
                joint_velocities = kinematic_tree->GetVelocityLimits();
            
                //Create a subscriber to allow external nodes to trigger a reset of robot's rest pose
                reset_rest_pose_sub = nh_.subscribe("/reset_rest_pose",10,&MotionPlannerBaseClass::ResetRestPoseCb,this);
            
                int T = problem->get_T();
                Eigen::VectorXd goal(6);
                goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                Eigen::VectorXdRefConst goal_ref = goal;
                for (int t = 0; t < T; ++t) {
                    problem->cost.SetGoal("Position", goal_ref, t);
                    problem->cost.SetRho("Position",1e1,t);
                }
                problem->cost.SetGoal("Position", goal_ref,-1);
                problem->cost.SetRho("Position",1e3,-1);
        
                solver->debug_ = false;
                solver->SetNumberOfMaxIterations(1);
                
            }
        }

    MotionPlannerBaseClass::~MotionPlannerBaseClass(){
        Setup::Destroy();   //Destroy all the exotica related objects
    }

    //Finds the current tf transform for the base and converts that to XYZ+RPY
    std::vector<double> MotionPlannerBaseClass::LookupBasePose() {
         geometry_msgs::TransformStamped base_transform;
         base_transform = tfBuffer.lookupTransform("odom","base",ros::Time(0),ros::Duration(3.0));
         std::vector<double> base_pose(6,0.0);

         //Extract translation components
         base_pose[0] = base_transform.transform.translation.x; 
         base_pose[1] = base_transform.transform.translation.y;
         base_pose[2] = base_transform.transform.translation.z;

         //Convert rotation components into RPY
         tf2::Quaternion q(
            base_transform.transform.rotation.x,
            base_transform.transform.rotation.y,
            base_transform.transform.rotation.z,
            base_transform.transform.rotation.w
         );
         tf2::Matrix3x3 m(q);
         double roll, pitch, yaw;
         m.getRPY(roll,pitch,yaw);

         //Set rotation components
         base_pose[3] = roll;
         base_pose[4] = pitch;
         base_pose [5] = yaw;
         return base_pose; //This vector effectively describes the base pose in terms of the 'odom' frame
    }

    void MotionPlannerBaseClass::SetJointLimits() {
        std::map<std::string,double> model_state;
        model_state["world_joint/trans_x"] = rest_pose[0];
        model_state["world_joint/trans_y"] = rest_pose[1];
        model_state["world_joint/trans_z"] = rest_pose[2];
        model_state["world_joint/rot_x"] = rest_pose[3];
        model_state["world_joint/rot_y"] = rest_pose[4];
        model_state["world_joint/rot_z"] = rest_pose[5];
        scene->SetModelState(model_state); //Sets the state of the base in EXOTica

        Eigen::VectorXd global_joint_limits_upper, global_joint_limits_lower;
        global_joint_limits_upper = relative_joint_limits_upper;
        global_joint_limits_lower = relative_joint_limits_lower;

        for (size_t i = 0; i < base_dof; i++) {
            global_joint_limits_upper(i) = (relative_joint_limits_upper[i] +
                                            rest_pose[i] + joint_limit_tolerance);
            global_joint_limits_lower(i) = (relative_joint_limits_lower[i] +
                                            rest_pose[i] - joint_limit_tolerance);
        }

        //Set joint limits in EXOTica scene
        kinematic_tree->SetJointLimitsUpper(global_joint_limits_upper);
        kinematic_tree->SetJointLimitsLower(global_joint_limits_lower);
    }

    void MotionPlannerBaseClass::ResetRestPoseCb(const std_msgs::Bool::ConstPtr &msg) {
        if (msg->data == true) {
            rest_pose = LookupBasePose(); //Resets the rest pose
            SetJointLimits(); //Resets the joint limits according to new rest pose
        }
    }

    //Initializes the robot pose in EXOTica
    void MotionPlannerBaseClass::InitRobotPose() {
        q = Eigen::VectorXd(problem->GetStartState()); //Ensures that q is a copy of the returned vector
        if ((joint_names.size() - base_dof) != 0 ) {
            //Wait for a joint state message from the arm and call the callback function
            boost::shared_ptr<const sensor_msgs::JointState> joint_state_msg_ptr;
            joint_state_msg_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/z1_gazebo/joint_states");
            JointStatesCb(*joint_state_msg_ptr);

            //If base DOF is not zero, update the base pose in q
            if (base_dof != 0) {
                std::vector<double> base_pose = LookupBasePose();
                for (size_t i = 0; i < base_dof; i++) {
                    q(i) = base_pose[i];
                }
                Eigen::MatrixXd joint_limits = kinematic_tree->GetJointLimits();
                //Clamp q to within constraints
                for (size_t i = 0; i < base_dof; i++) {
                    if (q(i) > joint_limits(i,1)) {q(i) = joint_limits(i,1);}
                    else if (q(i) < joint_limits(i,0)) {q(i)= joint_limits(i,0);}   
                }
                int nq =  scene->get_num_positions();
                Eigen::VectorXd q_part = q.head(nq);

                scene->SetModelState(q_part);
                problem->Update(scene->GetControlledState(),0);
                scene->GetKinematicTree().PublishFrames();
                
            } 
        }
    }
    //Publish motion plan as a single waypoint trajectory_msgs/JointTrajectory
    void MotionPlannerBaseClass::PublishToRobot() {
        trajectory_msgs::JointTrajectory trajectory_msg;
        trajectory_msg.joint_names = joint_names;
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions.resize(12); //Ensure the vector has 12 elements
        //Convert from Eigen::VectorXd to std::vector
        for (int i = 0; i < 12; i++) {trajectory_point.positions[i] = q(i);}
        ros::Duration duration;
        trajectory_point.time_from_start = duration.fromSec(dt);
        trajectory_msg.points.push_back(trajectory_point);
        motion_plan_publisher.publish(trajectory_msg);
    }

    double MotionPlannerBaseClass::GetError() {
        //A weird implementation going on here...
        double state_cost = problem->GetStateCost(0);
        double square_error = state_cost * state_cost;
        return std::sqrt(square_error);

    }

    void MotionPlannerBaseClass::JointStatesCb(const sensor_msgs::JointState &joint_states){
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

    void MotionPlannerBaseClass::InteractiveServoing() {
        std::vector<double> update_xy = LookupBasePose();
        std::map<std::string, double> model_state_xy;
        model_state_xy["world_joint/trans_x"] = update_xy[0];
        model_state_xy["world_joint/trans_y"] = update_xy[1];
        scene->SetModelState(model_state_xy);
        q(0) = update_xy[0];
        q(1) = update_xy[1];
        Eigen::MatrixXd joint_limits = kinematic_tree->GetJointLimits();
        Eigen::MatrixXd global_joint_limits_upper = joint_limits.col(1);
        Eigen::MatrixXd global_joint_limits_lower = joint_limits.col(0);

        global_joint_limits_upper(0) = (
            update_xy[0] + relative_joint_limits_upper(0) + joint_limit_tolerance
        );
        global_joint_limits_upper(1) = (
            update_xy[1] + relative_joint_limits_upper(1) + joint_limit_tolerance
        );
        global_joint_limits_lower(0) = (
            update_xy[0] + relative_joint_limits_lower(0) - joint_limit_tolerance
        );
        global_joint_limits_lower(1) = (
            update_xy[1] + relative_joint_limits_lower(1) - joint_limit_tolerance
        );

        kinematic_tree->SetJointLimitsUpper(global_joint_limits_upper);
        kinematic_tree->SetJointLimitsLower(global_joint_limits_lower);
    }

    void MotionPlannerBaseClass::Iterate() {
        //Inform EXOTica of unexpected changes to XY as a side-effect of ANYNova rotating its base
        InteractiveServoing();
        problem->SetStartTime(t); //Set problem start time
        problem->SetStartState(q); //Set problem start state

        //Solve using MPC
        Eigen::MatrixXd solution;
        solver->Solve(solution);
        
        problem->Update(q,solution.row(0),0);

        //Update Joint Positions
        q = problem->get_X().col(1);
        scene->SetModelState(q.head(scene->get_num_positions()));
    }

    
    KDL::Frame MotionPlannerBaseClass::GetFrameFromPose(const geometry_msgs::Pose &pose) {
        
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




