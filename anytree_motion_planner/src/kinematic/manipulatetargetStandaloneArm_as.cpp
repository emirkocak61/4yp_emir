
#include <ros/ros.h>
#include <exotica_core/trajectory.h>
#include <bt_drs_msgs/manipulateTargetAction.h>
#include <bt_drs_msgs/manipulateTargetFeedback.h>
#include <bt_drs_msgs/manipulateTargetResult.h>
#include <actionlib/server/simple_action_server.h>
#include <anytree_motion_planner/StandaloneArmKinematicPlanner.hpp>

class ManipulateTargetStandaloneArmActionServer : public StandaloneArmKinematicPlanner {
public:
    
    ManipulateTargetStandaloneArmActionServer() 
    :  StandaloneArmKinematicPlanner("manipulateTarget"),
    robot_name("unitree"),
    as_(nh_,action_name + "_as", boost::bind(&ManipulateTargetStandaloneArmActionServer::execute_cb,this, _1), false) {
        as_.start();
        tolerance_ = 1e-2;
    }

    void execute_cb(const bt_drs_msgs::manipulateTargetGoalConstPtr &goal) {
        //Define target frame
        KDL::Frame T_approach;
        T_approach = GetFrameFromPose(goal->target);

        //Setup Goal
        SetupGoal(T_approach);
        ROS_INFO("Manipulating target"); 
        std::shared_ptr<Trajectory> trajectory = std::make_shared<Trajectory>(DefineTrajectory(goal));
        PerformTrajectory(trajectory);
    }

    void PerformTrajectory(const  std::shared_ptr<Trajectory> &trajectory) override {
        result_.result = true;
        //InitRobotPose();
        scene->AddTrajectory("TargetRelative",trajectory);
        t = 0.0;
        Eigen::MatrixXd data = trajectory->GetData();
        t_limit = data(data.rows()-1,0) + 15; //Add a constant as an error margin

        //Check that EEF is within tolerance of the start waypoint
        problem->SetStartTime(t);
        Iterate();
        double error = GetManipulationError(); //Calculates error
        
        if (error > tolerance_) {
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
            max_increment = 0.005;
            manipulation_todo = goal->manipulation_todo;
            direction = goal->direction;
            if (goal->strategy == 0) {
                double time_stamp = 0.0;
                double manipulation_done = 0.0;
                //Now define the manipulation trajectory
                while (std::abs(manipulation_done) < std::abs(manipulation_todo)) {
                    time_stamp += dt;
                    manipulation_done += max_increment * direction;
                    Eigen::VectorXd point(7);
                    point << time_stamp, 0.0, 0.0, -0.097, 0.0, 0.0, manipulation_done - 1.5708;
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

protected:
    std::string robot_name;
    actionlib::SimpleActionServer<bt_drs_msgs::manipulateTargetAction> as_;
    bt_drs_msgs::manipulateTargetFeedback feedback_;
    bt_drs_msgs::manipulateTargetResult result_;

    //Objects related to manipulation
    double manipulation_todo;
    double max_increment;
    int8_t direction;

};

int main(int argc,char** argv) {
    ros::init(argc,argv,"manipulateTarget");
    ManipulateTargetStandaloneArmActionServer s; //Construct action server
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}
