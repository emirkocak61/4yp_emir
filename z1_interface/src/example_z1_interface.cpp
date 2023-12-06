#include "ros/ros.h"
#include "unitree_arm_sdk/control/unitreeArm.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"

using namespace UNITREE_ARM;

class Z1InterfaceNode : public unitreeArm {
public:
    ros::NodeHandle nh_;  // ROS node handle for managing subscriptions and publications
    ros::Subscriber sub;  // Subscriber to receive pose goals
    double gripper_pos = 0.0;  // Initial gripper position
    double joint_speed = 2.0;  // Speed for joint movement

    // Constructor: initializes the Unitree Arm and sets up the ROS subscriber
    Z1InterfaceNode() : unitreeArm(true) {
        // Subscribe to "z1_pose_goal" topic to receive pose goals
        ROS_INFO_STREAM("Z1_Interface running, waiting for goal commands...");
        sub = nh_.subscribe("z1_pose_goal", 10, &Z1InterfaceNode::move_arm, this);
    };

    // Callback function for the "z1_pose_goal" topic
    void move_arm(const geometry_msgs::PosePtr& msg) {
        ROS_INFO_STREAM("Received goal, sending to Z1 controllers");
        double roll, pitch, yaw;  // Variables for roll, pitch, and yaw
        tf::Quaternion quat;  // Quaternion for orientation

        // Convert the orientation from quaternion to RPY (roll, pitch, yaw)
        tf::quaternionMsgToTF(msg->orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Set the posture for the arm based on the received pose message
        Vec6 posture;
        posture << roll, pitch, yaw, msg->position.x, msg->position.y, msg->position.z;

        // Command the arm to move to the specified posture
        MoveJ(posture, gripper_pos, joint_speed);
    }

    ~Z1InterfaceNode() {}
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "z1_interface_node");
    Z1InterfaceNode arm;
    arm.sendRecvThread->start();
    arm.backToStart();
    
    ros::spin();

    return 0;
}
