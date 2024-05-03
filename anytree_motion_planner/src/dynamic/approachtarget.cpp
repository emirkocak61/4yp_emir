#include "actionlib/server/simple_action_server.h"
#include "bt_drs_msgs/approachTargetAction.h"
#include "bt_drs_msgs/approachTargetResult.h"
#include "bt_drs_msgs/approachTargetFeedback.h"
#include <anytree_motion_planner/MotionPlannerBaseClass.hpp>

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
        int gradient_counter = 0;

        while (counter < counter_limit && t < t_limit){
            if (as_.isPreemptRequested()) {
                result_.result = false;
                ROS_WARN("%s: PREEMPTED (Goal Cancelled)", action_name.c_str());
                as_.setPreempted();
                break;
            }

            Iterate(); //Generate motion plan via EXOTica
            PublishToRobot(); //Send motion plan to the robot
            double error = GetError(); //Calculate error

            //Publish feedback to client
            feedback_.error = error;
            as_.publishFeedback(feedback_); 

            //Check whether to increment the counter
            if (error < tolerance_) {counter++;}

            //Check whether to increment the gradient counter
            if (abs(error - previous_error) < gradient_tolerance) {gradient_counter++;}
            else {gradient_counter = 0;}

            previous_error = error;

            if (gradient_counter >= gradient_counter_limit) {break;}

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
            else if (gradient_counter >= gradient_counter_limit && error < convergence_tolerance){
                ROS_INFO("%s: SUCCESS (Converged within tolerance of Target", action_name.c_str());
                as_.setSucceeded(result_);
            }
            else if (gradient_counter >= gradient_counter_limit && error >= convergence_tolerance){
                ROS_WARN("%s: FAILURE (Converged outside tolerance of Target", action_name.c_str());
                as_.setAborted(result_);
            }
            else {
                result_.result = false;
                ROS_WARN("%s ABORTED (Did not reach Target within time limit)", action_name.c_str());
                as_.setAborted(result_);
            }
        }
        problem->SetStartTime(0.0); //Reset problem start time
    }
protected:
    std::string robot_name;
    actionlib::SimpleActionServer<bt_drs_msgs::approachTargetAction> as_;
    bt_drs_msgs::approachTargetFeedback feedback_;
    bt_drs_msgs::approachTargetResult result_;

};

int main(int argc,char** argv) {
    ros::init(argc,argv,"approachTarget");
    bool debug_motion_plan = false;
    if (ros::param::has("/debug_motion_plan")) {
        ros::param::get("/debug_motion_plan", debug_motion_plan);
    }
    if (debug_motion_plan) {Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));}
    ApproachTargetActionServer s; //Construct action server
    if (!debug_motion_plan) {
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }
    else {ros::waitForShutdown();}
}