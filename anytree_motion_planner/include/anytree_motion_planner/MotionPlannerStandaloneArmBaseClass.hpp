#ifndef MOTION_PLANNER_STANDALONE_ARM_BASE_CLASS_H
#define MOTION_PLANNER_STANDALONE_ARM_BASE_CLASS_H

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
#include <fstream>

using namespace exotica;
/**
 * @brief Base class for an MPC and EXOTica based motion planner.
 * Implemented as a ROS action server. This is an abstract base class, and specific action
 * implementations need to define and start the action server, feedback and result action
 * variables, and the callback for the action server. 
*/
class MotionPlannerStandaloneArmBaseClass {
public:
    /**
     * @brief Constructs a MotionPlannerStandaloneArmBaseClass object. 
     * @param actionName The name of the ROS action
    */
    MotionPlannerStandaloneArmBaseClass(const std::string& actionName);
    /**
     * @brief Class destructor. Also destroys all EXOTica related objects. 
    */
    virtual ~MotionPlannerStandaloneArmBaseClass();

    /**
     * @brief Initializes the robot pose in EXOTica based on the current joint states
    */
    void InitRobotPose();
    /**
     * @brief Publishes the motion plan as a single waypoint trajectory_msgs::JointTrajectory
    */
    void PublishToRobot();
    /**
     * @brief Calculates the error between the desired state and the current state
     * @return double The calculated error
    */
    double GetError();
    /**
     * @brief Callback for updating joint states from sensor_msgs::JointState messages
     * @param joint_states The received joint states message. 
    */
    void JointStatesCb(const sensor_msgs::JointState &joint_states);
    /**
     * @brief Performs one iteration of MPC solution and updates EXOTica problem
    */
    void Iterate();
    /**
     * @brief Converts a geometry_msgs::Pose to a KDL Frame 
    */
    KDL::Frame GetFrameFromPose(const geometry_msgs::Pose &pose);
    /**
     * @brief Virtual function to perform motion. Must be implemented in the derived class
    */
    virtual void PerformMotion() {};
    virtual void PerformTrajectory(const  std::shared_ptr<Trajectory> &trajectory) {};
    void RobotStateCb(const std_msgs::Float64MultiArrayConstPtr &state);
   

protected:
    ros::NodeHandle nh_; //Initialize the node handle first to avoid ros::Time related issues
    std::string action_name; //Name of the ROS action
    double dt; //Iteration time-step 0.02 corresponds to 50Hz control rate
    ros::Rate rate; //Control frequency
    double start_tolerance; //Tolerance between the End Effector Frame and Target at the start of motion plan
    std::string error_metric;
    double tolerance_; //Tolerance between EEF and Target
    int counter_limit; //No of consecutive iterations for which EEF is within tolerance
    double t_limit; //Time limit to achieve goal 
    double t; //Time in exotica planning problem
    Eigen::VectorXd q; //Joint states of the robot
    Eigen::VectorXd robot_state;
    //If the real arm can't follow the motion plans for manipulation, the motion plan should be stopped
    double self_tolerance;
    double self_counter; //No of consecutive iterations for which real EEF is outside tolerance

    //ROS related members
    ros::Publisher motion_plan_publisher; //ROS Publisher to  /motion_plan topic
    ros::Subscriber state_subscriber;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener; //Listens to published transforms from the robot sim

    //EXOTica related members
    MotionSolverPtr solver; //EXOTica Motion Solver 
    DynamicTimeIndexedShootingProblemPtr problem; //The EXOTica Planning Problem object
    ScenePtr scene; //The exotica planning scene
    std::vector<std::string> joint_names; //Names of the controlled joints
    KinematicTree* kinematic_tree; //The kinematic tree pointer from EXOTica scene
    double joint_limit_tolerance;
    Eigen::MatrixXd joint_velocities;

    std::unique_ptr<Eigen::MatrixXd> prevU; //Matrix to store previous control inputs
    std::mutex state_mutex;
    std::ofstream outFile; //This will be used to log the outputs of certain operations (testing)

};

#endif // MOTION_PLANNER_BASE_H