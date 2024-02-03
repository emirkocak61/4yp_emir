#include <anytree_motion_planner/TestAlgorithm.hpp>
#include <anytree_motion_planner/TestAICO.hpp>


int main(int argc,char** argv) {
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh_;
    
    {
        TestAlgorithm test;
        TestAICO aico;
        //Set the path to config file
        std::string config_path = "{anytree_motion_planner}/resources/configs/examples/example_timeindexed.xml";
        //Set the start state
        Eigen::VectorXd start_state(6);
        start_state << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0;
        std::cout << "Start state: " << start_state.transpose() << std::endl;
        //Set the target pose
        Eigen::VectorXd target_euler(6);
        target_euler << 0.7, 0.0, 0.7, -1.5708, 0.0, 1.5708;
        geometry_msgs::Pose target = test.GetPoseFromEuler(target_euler);
        KDL::Frame frame = test.GetFrameFromPose(target);
        aico.SetupProblem(config_path,start_state);
        aico.SetupGoal(frame);
        aico.Run();
        std::cout << "Problem solved" << std::endl;
        double time = aico.GetAverageTime();
        std::cout << "Average time to solve: " << time << std::endl;
        
        //test.SetupProblem(config_path,start_state,target);
    
        //double average_time = test.GetAverageTime();
    }
    ros::Duration(3.0).sleep();
    Setup::Destroy();

    return 0;
}