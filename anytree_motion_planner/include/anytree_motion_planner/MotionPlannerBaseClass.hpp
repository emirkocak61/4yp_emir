#ifndef MOTION_PLANNER_BASE_CLASS_H
#define MOTION_PLANNER_BASE_CLASS_H

#include "exotica_core/exotica_core.h" //Include exotica
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf2_ros/transform_listener.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "std_msgs/Float64.h"
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
class MotionPlannerBaseClass {
public:
    /**
     * @brief Constructs a MotionPlannerBaseClass object. 
     * @param actionName The name of the ROS action
    */
    MotionPlannerBaseClass(const std::string& actionName);
    /**
     * @brief Class destructor. Also destroys all EXOTica related objects. 
    */
    virtual ~MotionPlannerBaseClass();

    /**
     * @brief Callback for the base pose of the robot based on the ANYmal state estimator
     * @param pose_msg ROS message containing the base pose
    */
    void BasePoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    /**
     * @brief Sets the joint limits in the EXOTica scene based on the current rest pose
    */
    void SetJointLimits();
    /**
     * @brief Callback for resetting the rest pose of the robot based on the latest transform
     * @param msg ROS message indicating if the rest pose should be reset
    */
    void ResetRestPoseCb(const std_msgs::Bool::ConstPtr &msg);
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
     * @brief Updates robot's x and y position in EXOTica 
    */
    void InteractiveServoing();
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
    size_t base_dof; //Degree of freedom of the base, 6 in the case of virtual floating base
    double start_tolerance; //Tolerance between the End Effector Frame and Target at the start of motion plan
    double gradient_tolerance; //Tolerance between previous and current error, used for stopping upon convergence
    double convergence_tolerance; //Tolerance between End Effector Frame and Target once having stopped upon convergence
    std::string error_metric;
    double tolerance_; //Tolerance between EEF and Target
    int counter_limit; //No of consecutive iterations for which EEF is within tolerance
    int gradient_counter_limit; //No of consecutive iterations for which gradient is within tolerance
    double t_limit; //Time limit to achieve goal 
    double t; //Time in exotica planning problem
    double previous_error; //Records previous planning error (for computation of error gradient)
    Eigen::VectorXd q; //Joint states of the robot
    Eigen::VectorXd robot_state;
    std::vector<double> base_pose;
    //If the real arm can't follow the motion plans for manipulation, the motion plan should be stopped
    double self_tolerance;
    double self_counter; //No of consecutive iterations for which real EEF is outside tolerance

    //ROS related members
    ros::Publisher motion_plan_publisher; //ROS Publisher to  /motion_plan topic
    ros::Publisher error_publisher; //ROS Publisher to  /motion_plan topic
    ros::Subscriber state_subscriber;
    ros::Subscriber reset_rest_pose_sub; //ROS Subscriber to /reset_rest_pose topic
    ros::Subscriber base_pose_sub; // ROS Subscriber to state_estimator/pose_in_odom topic
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener; //Listens to published transforms from the robot sim

    //EXOTica related members
    MotionSolverPtr solver; //EXOTica Motion Solver 
    DynamicTimeIndexedShootingProblemPtr problem; //The EXOTica Planning Problem object
    ScenePtr scene; //The exotica planning scene
    std::vector<std::string> joint_names; //Names of the controlled joints
    std::vector<double> rest_pose; //Base pose of anymal at rest/beginning
    KinematicTree* kinematic_tree; //The kinematic tree pointer from EXOTica scene
    Eigen::VectorXd relative_joint_limits_upper;
    Eigen::VectorXd relative_joint_limits_lower;
    double joint_limit_tolerance;
    Eigen::MatrixXd joint_velocities;

    std::unique_ptr<Eigen::MatrixXd> prevU; //Matrix to store previous control inputs
    std::ofstream outFile; //This will be used to log the outputs of certain operations (testing)

};

#endif // MOTION_PLANNER_BASE_H