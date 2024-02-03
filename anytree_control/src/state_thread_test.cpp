#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_ros/transform_listener.h"
#include <thread>
#include <memory>

void getTransform(geometry_msgs::TransformStamped& transform_stamped,tf2_ros::Buffer& tfBuffer) {
    transform_stamped = tfBuffer.lookupTransform("odom","base",ros::Time(0),ros::Duration(3.0));
}
void getBasePose(geometry_msgs::PoseWithCovarianceStampedConstPtr& base_pose_ptr) {
    base_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/state_estimator/pose_in_odom");
}

void getBaseTwist(geometry_msgs::TwistWithCovarianceStampedConstPtr& base_twist_ptr) {
    base_twist_ptr = ros::topic::waitForMessage<geometry_msgs::TwistWithCovarianceStamped>("/state_estimator/twist");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_listener");
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    // Shared pointers for thread-safe access
    auto base_pose_ptr = std::make_shared<geometry_msgs::PoseWithCovarianceStampedConstPtr>();
    auto base_twist_ptr = std::make_shared<geometry_msgs::TwistWithCovarianceStampedConstPtr>();
    geometry_msgs::TransformStamped transform_stamped;

    // Launch threads to get base pose and twist in parallel
    std::thread transform_thread(getTransform, std::ref(transform_stamped),std::ref(tfBuffer));
    std::thread message_thread(getBasePose,std::ref(*base_pose_ptr));
    
    transform_thread.join();
    message_thread.join();

    // Now base_pose_ptr and base_twist_ptr should be populated
    if (*base_pose_ptr) {
        // Process the pose and twist
        // Example: print the received pose and twist
        std::cout << "Completed execution" << std::endl;
        ROS_INFO_STREAM("Received pose via message: " << (*base_pose_ptr)->pose.pose << " " <<(*base_pose_ptr)->header.stamp.nsec) ;
        ROS_INFO_STREAM("Received pose via transform : " << transform_stamped.transform << " " << transform_stamped.header.stamp.nsec);
    } else {
        ROS_ERROR("Did not receive pose or twist message.");
    }
    ros::spin();
}