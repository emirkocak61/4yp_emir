#include "actionlib/server/simple_action_server.h"
#include "bt_drs_msgs/approachTargetAction.h"
#include "bt_drs_msgs/approachTargetResult.h"
#include "bt_drs_msgs/approachTargetFeedback.h"
#include <anytree_motion_planner/MPCKinematicPlanner.hpp>

class ApproachTargetActionServer : public MPCKinematicPlanner {

public:
    ApproachTargetActionServer() 
        //Call the constructor of the base class
        : MPCKinematicPlanner("approachTarget"), 
        robot_name("anytree"),
        //Instantiate the action server
        as_(nh_, action_name + "_as", boost::bind(&ApproachTargetActionServer::execute_cb, this, _1), false)
    {
        //Initialize(); //Initialize ROS-time dependent objects. 
        as_.start();
        tolerance_ = 1e-2;
        
    }
    
    void execute_cb(const bt_drs_msgs::approachTargetGoalConstPtr &goal){
        //Define target frame
        KDL::Frame T_approach;
        T_approach = GetFrameFromPose(goal->target);
        //Attach target frame in absolute world frame
        SetupGoal(T_approach);
        ROS_INFO("Approaching target");
        PerformMotion();
    }


    void PerformMotion() override {
        result_.result = true;
        //InitRobotPose();
        t = 0.0;
        int counter = 0;
        double error;

        while (counter < counter_limit && t < time_limit){
            if (as_.isPreemptRequested()) {
                result_.result = false;
                ROS_WARN("%s: PREEMPTED (Goal Cancelled)", action_name.c_str());
                as_.setPreempted();
                break;
            }
            
            Iterate(); //Generate motion plan via EXOTica
            PublishToRobot(); //Send motion plan to the robot
            error = GetError(); //Calculate error
            std::cout << "Error: " << error << std::endl;
            //Publish feedback to client
            feedback_.error = error;
            as_.publishFeedback(feedback_); 

            //Check whether to increment the counter
            if ((error) < tolerance_) {counter++;}

            //Sleep and increment time
            rate.sleep();
            t = t + dt;  
        }

        //Check that EEF has reached target
        if (result_.result == true) {
            error = GetError();
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
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}

