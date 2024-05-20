
#include <ros/ros.h>
#include <exotica_core/trajectory.h>
#include <bt_drs_msgs/manipulateTargetAction.h>
#include <bt_drs_msgs/manipulateTargetFeedback.h>
#include <bt_drs_msgs/manipulateTargetResult.h>
#include <actionlib/server/simple_action_server.h>
#include <anytree_motion_planner/MotionPlannerBaseClass.hpp>

class ManipulateTargetActionServer : public MotionPlannerBaseClass {
public:
    
    ManipulateTargetActionServer() 
    : MotionPlannerBaseClass("manipulateTarget"),
    robot_name("anytree"),
    as_(nh_,action_name + "_as", boost::bind(&ManipulateTargetActionServer::execute_cb,this, _1), false) {
        gripper_pub = nh_.advertise<std_msgs::Bool>("/gripper_command",10);
        as_.start();
    }

    void execute_cb(const bt_drs_msgs::manipulateTargetGoalConstPtr &goal) {
        //Define target frame
        KDL::Frame T_approach;
        T_approach = GetFrameFromPose(goal->target);

        //Attach object in the absolute world frame
        scene->AttachObjectLocal("Target","",T_approach);
        ROS_INFO("Manipulating target"); 
        std::shared_ptr<Trajectory> trajectory = std::make_shared<Trajectory>(DefineTrajectory(goal));
        PerformTrajectory(trajectory);
    }

    void PerformTrajectory(const  std::shared_ptr<Trajectory> &trajectory) override {
        result_.result = true;
        std_msgs::Bool gripper_command;
        InitRobotPose();
        scene->AddTrajectory("TargetRelative",trajectory);
        t = 0.0;
        Eigen::MatrixXd data = trajectory->GetData();
        t_limit = data(data.rows()-1,0) + 5; //Add a constant as a safety margin

        //Check that EEF is within tolerance of the start waypoint
        problem->SetStartTime(t);
        Iterate();
        double error = GetManipulationError(); //Calculates error
        
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
                error = GetManipulationError();

                //Publish feedback to client
                feedback_.error = error;
                as_.publishFeedback(feedback_);

                //Sleep and increment time
                rate.sleep();
                t += dt;
            }

            //Check that EEF has reached final waypoint
            if(result_.result == true) {
                error = GetManipulationError();
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
    Trajectory DefineTrajectory(const bt_drs_msgs::manipulateTargetGoalConstPtr &goal) {
        //Use std::vector in order to add elements dynamically
        std::vector<Eigen::VectorXd> trajectoryPoints;
        Eigen::MatrixXd trajectory;
        if (goal->device_type == "needle_valve") {
            manipulation_todo = goal->manipulation_todo;
            direction = goal->direction;
            if (goal->strategy == 0) {
                max_increment = 0.005;
                double z_offset = 0.125;
                double time_stamp = 0.0;
                double manipulation_done = 0.0;
                //Now define the manipulation trajectory
                while (std::abs(manipulation_done) < std::abs(manipulation_todo)) {
                    time_stamp += dt;
                    manipulation_done += max_increment * direction;
                    Eigen::VectorXd point(7);
                    point << time_stamp, 0.0, 0.0, z_offset, 0.0, 0.0, 1.5708 + manipulation_done;
                    trajectoryPoints.push_back(point);
                }
            }
            else if (goal->strategy == 1) {
                max_increment = 0.001;
                double z_offset = 0.125;
                double phi = pi/5;
                double time_stamp = 0.0;
                double manipulation_done = 0.0;
                //Now define the manipulation trajectory
                while (std::abs(manipulation_done) < std::abs(manipulation_todo)) {
                    time_stamp += dt;
                    manipulation_done += max_increment * direction;
                    Eigen::VectorXd point(7);
                    point << time_stamp, -z_offset*sin(phi)*sin(manipulation_done), z_offset*sin(phi)*cos(manipulation_done), z_offset*cos(phi), 0.0, phi, 1.5708 + manipulation_done;
                    trajectoryPoints.push_back(point);
                }
            }
        }
        //Now convert the std::vector into Eigen Matrix
        size_t n = trajectoryPoints.size();
        int m = 7;
        trajectory = Eigen::MatrixXd::Zero(n,m);
        
        //Copy the manipulation trajectory
        for (size_t i = 0; i < n; i++) {
            trajectory.row(i) = trajectoryPoints[i];
        }

        Trajectory traj_exotica(trajectory,0.1);
        return traj_exotica;
    }

    double GetManipulationError() {
        double error = GetError();
        feedback_.manipulation_remaining = (
            manipulation_todo - (t * max_increment) / dt
        );
        //ROS_INFO("Manipulation Remaining: %f", feedback_.manipulation_remaining);
        feedback_.angle_rotated = (t * max_increment * direction / dt);
        return error;
    }

protected:
    std::string robot_name;
    actionlib::SimpleActionServer<bt_drs_msgs::manipulateTargetAction> as_;
    bt_drs_msgs::manipulateTargetFeedback feedback_;
    bt_drs_msgs::manipulateTargetResult result_;
    ros::Publisher gripper_pub;

    //Objects related to manipulation
    double manipulation_todo;
    double max_increment;
    int8_t direction;

};

int main(int argc,char** argv) {
    ros::init(argc,argv,"manipulateTarget");
    bool debug_motion_plan = false;
    if (ros::param::has("/debug_motion_plan")) {
        ros::param::get("/debug_motion_plan", debug_motion_plan);
    }
    if (debug_motion_plan) {Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));}
    ManipulateTargetActionServer s; //Construct action server
    if (!debug_motion_plan) {
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }
    else {ros::waitForShutdown();}
}