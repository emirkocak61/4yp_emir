#pragma once

#include <anytree_bt/bt_action_client_node.hpp>
#include <anytree_bt/custom_types.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <navigation_manager_msgs/NavigateToGoalAction.h>

/**
 * @brief Asynchronous ActionNode for setting navigation goals
*/

class NavigateToGoalAnymal : public ActionClientNode {
    typedef actionlib::SimpleActionClient<
    navigation_manager_msgs::NavigateToGoalAction> Client;

public:
    NavigateToGoalAnymal(const std::string& name, const BT::NodeConfig& config) :
        ActionClientNode(name,config) {
            client_ptr = std::unique_ptr<Client>(
                new Client("/path_planning_and_following/navigate_to_goal", true)
            );
        }
    ~NavigateToGoalAnymal() override {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<geometry_msgs::Pose>("target"),
                BT::InputPort<std::string>("frame_id"),
                BT::InputPort<Position3D>("position"),
                BT::InputPort<Quaternion>("orientation")
        };
    }

    void sendStartRequest() override {
        client_ptr->waitForServer();
        navigation_manager_msgs::NavigateToGoalGoal goal;

        //Optional input: [frame_id]. If not specified, assume [odom]
        auto frame_id = getInput<std::string>("frame_id");

        auto target = getInput<geometry_msgs::Pose>("target");
        auto position = getInput<Position3D>("position");
        auto orientation = getInput<Quaternion>("orientation");

        if (!target && !position && !orientation) {
            throw BT::RuntimeError(
                "missing required input for navigation goal, either [target or]"
                "[position] and [orientation] need to be provided"
            );
        }

        goal.goal.header.stamp = ros::Time::now();
        goal.goal.header.frame_id = frame_id ? frame_id.value() : "odom";

        if (target) {
            goal.goal.pose = target.value();
        } else {
            goal.goal.pose.position.x = position.value().x;
            goal.goal.pose.position.y = position.value().y;
            goal.goal.pose.position.z = position.value().z;
            goal.goal.pose.orientation.x = orientation.value().x;
            goal.goal.pose.orientation.y = orientation.value().y;
            goal.goal.pose.orientation.z = orientation.value().z;
            goal.goal.pose.orientation.w = orientation.value().w;

        }

        goal.goal.tolerance.translation = 0.01; // 1cm
        goal.goal.tolerance.rotation = 0.05;    // 0.05rad = ~2.9deg
        goal.route_option.data = any_navigation_msgs::RouteOption::STRAIGHT_LINE;

        if (target) {
        std::cout << "Requesting navigation to [" << target.value().position.x
                    << ", " << target.value().position.y << ", "
                    << target.value().position.z << "], ["
                    << target.value().orientation.x << ", "
                    << target.value().orientation.y << ", "
                    << target.value().orientation.z << ", "
                    << target.value().orientation.w << "]" << std::endl;
        } else {
        std::cout << "Requesting navigation to [" << position.value().x << ", "
                    << position.value().y << ", " << position.value().z << "], ["
                    << orientation.value().x << ", " << orientation.value().y
                    << ", " << orientation.value().z << ", "
                    << orientation.value().w << "]" << std::endl;
        }

        client_ptr->sendGoal(goal);
    }

     bool getResult() override {
        navigation_manager_msgs::NavigateToGoalResultConstPtr result =
            client_ptr->getResult();
        if (result->status !=
            navigation_manager_msgs::NavigateToGoalResult::GOAL_REACHED) {
        std::cerr << "Failed to navigate to goal (status code "
                    << static_cast<int>(result->status) << ")" << std::endl;
        }
        return (result->status ==
                navigation_manager_msgs::NavigateToGoalResult::GOAL_REACHED);
    }

    State getState() override {
        auto state = client_ptr->getState();
        if (state == State::StateEnum::ABORTED) {
        (void)getResult();
        }
        return state;
    }

    void sendAbortSignal() override { client_ptr->cancelGoal(); }
    

private:
    std::unique_ptr<Client> client_ptr;
};