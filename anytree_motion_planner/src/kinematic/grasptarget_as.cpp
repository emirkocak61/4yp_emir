//Action server to grasp the target 
#include <ros/ros.h>
#include <exotica_core/trajectory.h>
#include <bt_drs_msgs/graspTargetAction.h>
#include <bt_drs_msgs/graspTargetFeedback.h>
#include <bt_drs_msgs/graspTargetResult.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <anytree_motion_planner/MPCKinematicPlanner.hpp>

class GraspTargetActionServer : public MPCKinematicPlanner {
public:
    GraspTargetActionServer()
    //Call the constructor of the base class
    : MPCKinematicPlanner("graspTarget"),
    robot_name("anytree"),
    //Instantiate the action server
    as_(nh_,action_name + "_as", boost::bind(&GraspTargetActionServer::execute_cb,this, _1), false) {
        as_.start();
        tolerance_ = 2e-2;
    }

void execute_cb(const bt_drs_msgs::graspTargetGoalConstPtr &goal) {
    ros::Duration(1.0).sleep(); //Give time to open gripper
    //Define target frame
    KDL::Frame T_approach;
    T_approach = GetFrameFromPose(goal->target);

    //Attach object in the absolute world frame
    SetupGoal(T_approach);
    ROS_INFO("Grasping target"); 
    std::shared_ptr<Trajectory> trajectory = std::make_shared<Trajectory>(DefineTrajectory(goal));
    PerformTrajectory(trajectory);
}

Trajectory DefineTrajectory(const bt_drs_msgs::graspTargetGoalConstPtr &goal) {
    
    Eigen::MatrixXd trajectory;
    if (goal->device_type == "needle_valve") {
       if(goal->strategy == 0) {
        trajectory = Eigen::MatrixXd::Zero(4,7); //time + xyz + rpy
        trajectory.row(0) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //start
        trajectory.row(1) << 3.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0; //head on
        trajectory.row(2) << 6.0, 0.0, -0.0, 0.0, 0.0, 0.0, -1.5708; //rotate gripper
        trajectory.row(3) << 9.0, 0.0, -0.0, -0.097, 0.0, 0.0, -1.5708; //grasp
       } 
    }
    Trajectory traj_exotica(trajectory,0.1);
    return traj_exotica;
}

void SetupGoal(const KDL::Frame target_frame) override {
        scene->AttachObjectLocal("Target","", target_frame);
        int T = problem->GetT();
        Eigen::VectorXd goal(6);
        goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        double alpha = 0.999;
        double rho = 1e3;
        for (int t(0); t < T; t++) {
            rho = pow(alpha,t) * 1e4;
            problem->cost.SetGoal("Position", goal, t);
            problem->cost.SetRho("Position",rho,t);
        }
    }

void PerformTrajectory(const  std::shared_ptr<Trajectory> &trajectory) override {
    result_.result = true;
    InitRobotPose();
    scene->AddTrajectory("TargetRelative",trajectory);
    t = 0.0;
    Eigen::MatrixXd data = trajectory->GetData();
    t_limit = data(data.rows()-1,0) + 10; //Add a constant as an error margin;

    //Check that EEF is within tolerance of the start waypoint
    problem->SetStartTime(t);
    Iterate();
    double error = GetError(); //Calculates error
    std::cout << "Error: " << error << std::endl;
    
    if (error > 1) {
        result_.result = false;
        ROS_WARN("%s, ABORTED (EEF Start Pose beyond tolerance)", action_name.c_str());
        as_.setAborted(result_);
    } 
    else{
        //If start pose check succeded initialise robot pose
        //InitRobotPose();
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
            std::cout << "Error: " << error << std::endl;

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
            if (error < 0.5) {
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
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}
