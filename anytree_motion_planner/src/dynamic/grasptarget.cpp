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
        gripper_command_publisher = nh_.advertise<std_msgs::Bool>("/gripper_command",10);
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
        trajectory.row(0) << 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.5708; //start
        trajectory.row(1) << 1.0, 0.015, 0.0, 0.2, 0.0, 0.0, 1.5708; //head on (distances stator finger from handle)
        trajectory.row(2) << 6.0, 0.015, 0.0, 0.135, 0.0, 0.0, 1.5708; //grasp (distances stator finger from handle)
        trajectory.row(3) << 7.0, 0.0, 0.0, 0.135, 0.0, 0.0, 1.5708; //grasp (moves stator finger to handle)
        trajectory.row(4) << 9.0, 0.0, 0.0, 0.135, 0.0, 0.0, 1.5708; //hold 
       } 
    }
    Trajectory traj_exotica(trajectory,1.0);
    return traj_exotica;
}

void PerformTrajectory(const  std::shared_ptr<Trajectory> &trajectory) override {
    result_.result = true;
    InitRobotPose();
    scene->AddTrajectory("TargetRelative",trajectory);
    t = 0.0;
    Eigen::MatrixXd data = trajectory->GetData();
    t_limit = data(data.rows()-1,0);
    std_msgs::Bool gripper_goal;

    //Check that EEF is within tolerance of the start waypoint
    problem->SetStartTime(t);
    Iterate();
    double error = GetError(); //Calculates error
    
    if (error > start_tolerance) {
        result_.result = false;
        ROS_WARN("%f > %f", error, start_tolerance);
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
    ros::Publisher gripper_command_publisher;
};

int main(int argc,char** argv) {
    ros::init(argc,argv,"graspTarget");
    bool debug_motion_plan = false;
    if (ros::param::has("/debug_motion_plan")) {
        ros::param::get("/debug_motion_plan", debug_motion_plan);
    }
    if (debug_motion_plan) {Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));}
    GraspTargetActionServer s; //Construct action server
    if (!debug_motion_plan) {
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }
    else {ros::waitForShutdown();}
}