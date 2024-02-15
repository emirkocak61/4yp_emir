#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "bt_drs_msgs/approachTargetAction.h"
#include "bt_drs_msgs/approachTargetResult.h"
#include "bt_drs_msgs/approachTargetFeedback.h"
#include "exotica_core/exotica_core.h"
#include "anytree_motion_planner/MotionPlannerBaseClass.hpp"

class ApproachTargetActionServer : public MotionPlannerBaseClass {

public:
    ApproachTargetActionServer() 
        //Call the constructor of the base class
        : MotionPlannerBaseClass("approachTarget"), 
        robot_name("anytree"),
        //Instantiate the action server
        as_(nh_, action_name + "_as", boost::bind(&ApproachTargetActionServer::execute_cb, this, _1), false)
    {
        //Initialize(); //Initialize ROS-time dependent objects. 
        as_.start();
        
    }
    
    void execute_cb(const bt_drs_msgs::approachTargetGoalConstPtr &goal){
        //Define target frame
        KDL::Frame T_approach;
        T_approach = GetFrameFromPose(goal->target);

        //Attach target frame in absolute world frame
        scene->AttachObjectLocal("Target","",T_approach);
        ROS_INFO("Approaching target");
        PerformMotion();
    }

    void PerformMotion() override {
        result_.result = true;
        InitRobotPose();
        t = 0.0;
        int counter = 0;

        while (counter < counter_limit && t < 20){
            if (as_.isPreemptRequested()) {
                result_.result = false;
                ROS_WARN("%s: PREEMPTED (Goal Cancelled)", action_name.c_str());
                as_.setPreempted();
                break;
            }
            Eigen::VectorXd state_error = q - robot_state;
            std::cout << "State error norm: " << state_error.norm() << std::endl;
            error_vector.push_back(state_error.norm());
            Iterate(); //Generate motion plan via EXOTica
            PublishToRobot(); //Send motion plan to the robot
            double error = GetError(); //Calculate error

            //Publish feedback to client
            feedback_.error = error;
            as_.publishFeedback(feedback_); 

            //Check whether to increment the counter
            if (error < tolerance_) {counter++;}

            //Sleep and increment time
            rate.sleep();
            t = t + dt;  
        }

        //Check that EEF has reached target
        if (result_.result == true) {
            double error = GetError();
            if (error < tolerance_){
                ROS_INFO("%s: SUCCESS (Approached Target)", action_name.c_str());
                as_.setSucceeded(result_);
            }
            else {
                result_.result = false;
                ROS_WARN("%s ABORTED (Did not reached target within time limit)", action_name.c_str());
                as_.setAborted(result_);
            }
        }
        problem->SetStartTime(0.0); //Reset problem start time
        double mean_error = 0;
        for (size_t i = 0; i < error_vector.size(); i++) {mean_error += error_vector[i];}
        mean_error = mean_error/(static_cast<double>(error_vector.size()));
        std::cout << "Mean state error: " << mean_error << std::endl;
    }
protected:
    std::string robot_name;
    actionlib::SimpleActionServer<bt_drs_msgs::approachTargetAction> as_;
    bt_drs_msgs::approachTargetFeedback feedback_;
    bt_drs_msgs::approachTargetResult result_;
    std::vector<double> error_vector;
};

int main(int argc,char** argv) {

    ros::init(argc,argv,"ApproachTarget");
    ApproachTargetActionServer s; //Construct action server
    ros::spin();
}

