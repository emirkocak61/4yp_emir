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
        start_state << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0;
        std::cout << "Start state: " << start_state.transpose() << std::endl;
        //Set the target pose
        Eigen::VectorXd target_euler(6);
        target_euler << 0.7, 0.0, 0.7, -1.5708, 0.0, 1.5708;
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
        test.SetupProblem(config_path,start_state, target);
        test.RunAlgorithm(8);
        double average_time = test.GetAverageTime();
        std::cout << "Average time to solve " << average_time << " (microseconds)" << std::endl;
    }
    ros::Duration(3.0).sleep();
    Setup::Destroy();

    return 0;
}