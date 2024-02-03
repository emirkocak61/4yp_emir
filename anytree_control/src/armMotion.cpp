
#include <anytree_motion_planner/TestAlgorithm.hpp>
#include <anytree_control/ArmControl.hpp>


int main(int argc,char** argv) {
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh_;
    {
    ArmControl controller;
    ros::Duration(2.0).sleep();
    std::string config_path = "{anytree_control}/resources/configs/arm_control.xml";
    //Set the start state
    Eigen::VectorXd start_state(6);
    start_state << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0;
    Eigen::VectorXd target_euler(6);
    target_euler << 0.7, 0.0, 0.7, -1.5708, 0.0, 1.5708;
    geometry_msgs::Pose target_pose = TestAlgorithm::GetPoseFromEuler(target_euler);
    KDL::Frame target_frame = TestAlgorithm::GetFrameFromPose(target_pose);

    controller.SetupPlanningProblem(config_path);
    controller.SetGoal(target_frame);
    controller.Run();
    }
    exotica::Setup::Destroy();
    return 0;
}

