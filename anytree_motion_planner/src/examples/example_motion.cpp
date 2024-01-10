#include <anytree_motion_planner/TestAlgorithm.hpp>
#include <ros/ros.h>

int main(int argc,char** argv) {
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh_;
    
    {
        TestAlgorithm test;
        //Set the path to config file
        std::string config_path = "{anytree_motion_planner}/resources/configs/examples/example_mpc.xml";
        //Set the start state
        Eigen::VectorXd start_state(6);
        start_state << -1.47, 3.45, 5.28, 0.0 ,0.0 , 0.0;
        std::cout << "Start state: " << start_state.transpose() << std::endl;
        //Set the target pose
        Eigen::VectorXd target_euler(6);
        target_euler << 0.6, 0.1, 0.4, 0, 0.0, 0;
        geometry_msgs::Pose target = test.GetPoseFromEuler(target_euler);
        Eigen::VectorXd target_pose(7);
        target_pose <<  target.position.x, 
                        target.position.y,
                        target.position.z,
                        target.orientation.x,
                        target.orientation.y,
                        target.orientation.z,
                        target.orientation.w;
        
        std::cout << "Target Pose: " << target_pose.transpose() << std::endl;
        test.SetupProblem(config_path,start_state, target_pose);
        //test.SetGoal(target_euler);
        test.RunAlgorithm(15);
    }
    ros::Duration(3.0).sleep();
    Setup::Destroy();

    return 0;
}