//Action server to grasp the target 
#include <ros/ros.h>
#include <exotica_core/trajectory.h>
#include <bt_drs_msgs/graspTargetAction.h>
#include <bt_drs_msgs/graspTargetFeedback.h>
#include <bt_drs_msgs/graspTargetResult.h>
#include <actionlib/server/simple_action_server.h>
#include <anytree_motion_planner/MotionPlannerBaseClass.hpp>

class GraspTargetActionServer : public MotionPlannerBaseClass {
public:
    GraspTargetActionServer()
    //Call the constructor of the base class
    : MotionPlannerBaseClass("graspTarget"),
    robot_name("anytree"),
    //Instantiate the action server
    as_(nh_,action_name + "_as", boost::bind(&GraspTargetActionServer::execute_cb,this, _1), false) {
        as_.start();
    }

void execute_cb(const bt_drs_msgs::graspTargetGoalConstPtr &goal) {
    //Define target frame
    KDL::Frame T_approach;
    T_approach = GetFrameFromPose(goal->target);

    //Attach object in the absolute world frame
    scene->AttachObjectLocal("Target","",T_approach);
    ROS_INFO("Grasping target"); 
    std::shared_ptr<Trajectory> trajectory = std::make_shared<Trajectory>(DefineTrajectory(goal));
    PerformTrajectory(trajectory);
}

Trajectory DefineTrajectory(const bt_drs_msgs::graspTargetGoalConstPtr &goal) {
    
    Eigen::MatrixXd trajectory;
    if (goal->device_type == "needle_valve") {
       if(goal->strategy == 0) {
        trajectory = Eigen::MatrixXd::Zero(5,7); //time + xyz + rpy
        trajectory.row(0) << 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0; //start
        trajectory.row(1) << 2.0, 0.015, 0.0, 0.3, 0.0, 0.0, 1.5708; //head on (distances stator finger from handle)
        trajectory.row(2) << 6.0, 0.015, 0.0, 0.190, 0.0, 0.0, 1.5708; //grasp (distances stator finger from handle)
        trajectory.row(3) << 7.0, 0.0, 0.0, 0.190, 0.0, 0.0, 1.5708; //grasp (moves stator finger to handle)
        trajectory.row(4) << 9.0, 0.0, 0.0, 0.190, 0.0, 0.0, 1.5708; //hold 
       } 
       else if(goal->strategy == 1){
        double phi = pi/5;
        trajectory = Eigen::MatrixXd::Zero(5,7); //time + xyz + rpy
        trajectory.row(0) << 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 1.5708; //start
        trajectory.row(1) << 1.0, 0.015, 0.0, 0.3, 0.0, 0.0, 1.5708; //head on (distances stator finger from handle)
        trajectory.row(2) << 10.0, 0.015, -0.138*sin(phi), 0.138*cos(phi), 0.0, -phi, 1.5708; //grasp (distances stator finger from handle)
        trajectory.row(3) << 11.0, 0.0, -0.138*sin(phi), 0.138*cos(phi), 0.0, -phi, 1.5708; //grasp (moves stator finger to handle)
        trajectory.row(4) << 13.0, 0.0, -0.138*sin(phi), 0.138*cos(phi), 0.0, -phi, 1.5708; //hold 
       }
    }

    else if (goal->device_type == "button"){
        if (goal->strategy == 0) {
            if (goal->direction == 1) { //Pushing the button
                trajectory = Eigen::MatrixXd::Zero(3,7); //time + xyz + rpy 
                trajectory.row(0) << 0.5, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0; //start
                trajectory.row(1) << 3.0, 0.0, -0.0, 0.250, 0.0, 0.0, 0.0; // 
                trajectory.row(2) << 6.0, 0.0, 0.0, 0.225,0.0, 0.0, 0.0;
            }
            else if (goal->direction == -1) {   //Releasing the button
                trajectory = Eigen::MatrixXd::Zero(4,7); //time + xyz + rpy 
                trajectory.row(0) << 0.5, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0; //start
                trajectory.row(1) << 2.0, 0.0, -0.021, 0.225, 0.0, 0.0, 0.0; //get closer from below
                trajectory.row(2) << 6.0, 0.0, -0.021, 0.187,0.0, 0.0, 0.0; //Get really close
                trajectory.row(3) << 8.0, 0.0, -0.017, 0.187,0.0, 0.0, 0.0; //move slightly upwards for grasping with the non-actuated gripper  
            }
        }    
    }
    else if (goal->device_type == "DN40_globe_valve") {
        double radius = 0.086;
        int direction = goal->direction;
        if (goal->strategy == 0) {
            trajectory = Eigen::MatrixXd::Zero(4,7); //time + xyz + rpy
            trajectory.row(0) << 0.5, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0; //start
            trajectory.row(1) << 3.0, 0.0, radius - 0.02, 0.250, 0.0, 0.0, 0.0; //Head on
            trajectory.row(2) << 6.0, 0.0, radius - 0.02, 0.135, 0.0, 0.0, 0.0; //grasp
            trajectory.row(3) << 7.0, 0.0, radius - 0.017, 0.135, 0.0, 0.0, 0.0; //hold
        }

        else if (goal->strategy == 1) {
            double offset = 0.7854; // 45 degrees
            trajectory = Eigen::MatrixXd::Zero(4,7); //time + xyz + rpy
            trajectory.row(0) << 0.5, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0; //start
            trajectory.row(1) << 3.0, 0.0, -radius, 0.250, 0.0, 0.0, 0.0; //Head on
            trajectory.row(2) << 6.0, radius/2 + 0.019, -radius + 0.019, 0.135, 0.0, 0.0, offset; //grasp
            trajectory.row(3) << 7.0, radius/2 + 0.018, -radius + 0.019 , 0.135, 0.0, 0.0, offset; //hold
        }
    }
    
    Trajectory traj_exotica(trajectory,0.1);
    return traj_exotica;
}

void PerformTrajectory(const  std::shared_ptr<Trajectory> &trajectory) override {
    result_.result = true;
    InitRobotPose();
    scene->AddTrajectory("TargetRelative",trajectory);
    t = 0.0;
    Eigen::MatrixXd data = trajectory->GetData();
    t_limit = data(data.rows()-1,0) + 1; //Add a constant as a safety margin

    //Check that EEF is within tolerance of the start waypoint
    problem->SetStartTime(t);
    Iterate();
    double error = GetError(); //Calculates error
    
    if (error > start_tolerance) {
        result_.result = false;
        ROS_WARN("%s, ABORTED (EEF Start Pose beyond tolerance)", action_name.c_str());
        as_.setAborted(result_);
    } 
    else{
        //If start pose check succeded initialise robot pose
        InitRobotPose();
        while(t < t_limit) {
            if (as_.isPreemptRequested()) {
                result_.result = false;
                ROS_WARN("%s: PREEMPTED (Goal Cancelled)", action_name.c_str());
                as_.setPreempted();
                break;
            }

            Iterate();
            PublishToRobot();
            error = GetError();

            //Publish feedback to client
            feedback_.error = error;
            as_.publishFeedback(feedback_);

            //Sleep and increment time
            rate.sleep();
            t += dt;
        }
        //Check that EEF has reached final waypoint
        if(result_.result == true) {
            error = GetError();
            if (error < tolerance_) {
                ROS_INFO("%s: SUCCESS (Completed Trajectory)", action_name.c_str());
                as_.setSucceeded(result_);
            } else {
                result_.result = false;
                ROS_WARN("%s: ABORTED (Did not reach final waypoint Within Time Limit)", action_name.c_str());
                as_.setAborted(result_);
            }
        }
    }


    problem->SetStartTime(0.0);
    scene->RemoveTrajectory("TargetRelative");
}
protected:
    std::string robot_name;
    actionlib::SimpleActionServer<bt_drs_msgs::graspTargetAction> as_;
    bt_drs_msgs::graspTargetFeedback feedback_;
    bt_drs_msgs::graspTargetResult result_;
};

int main(int argc,char** argv) {

    ros::init(argc,argv,"graspTarget");
    GraspTargetActionServer s; //Construct action server
    ros::spin();
}