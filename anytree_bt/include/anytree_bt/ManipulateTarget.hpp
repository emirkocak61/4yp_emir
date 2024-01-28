#pragma once

#include <ros/ros.h>
#include <anytree_bt/bt_action_client_node.hpp>
#include <anytree_bt/custom_types.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <bt_drs_msgs/manipulateTargetAction.h>
#include <bt_drs_msgs/manipulateTargetGoal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ManipulateTarget : public ActionClientNode {
    
    typedef actionlib::SimpleActionClient<
        bt_drs_msgs::manipulateTargetAction> Client;
public:
    ManipulateTarget(const std::string &name, const BT::NodeConfig &config) :
        ActionClientNode(name, config) {
            client_ptr = std::unique_ptr<Client>(
                new Client("manipulateTarget_as",true)
            );
        }
    
    static BT::PortsList providedPorts() {
        return {BT::InputPort<Position3D>("position"),
                BT::InputPort<Orientation3D>("orientation_3D"),
                BT::InputPort<std::string>("device_type"),
                BT::InputPort<int>("strategy"),
                BT::InputPort<int>("direction"),
                BT::InputPort<double>("manipulation_todo")};
    }
    
    geometry_msgs::Pose GetPoseFromEuler(const Position3D &position,
                                         const Orientation3D& orientation) {
        geometry_msgs::Pose pose;
        //Set translation components
        pose.position.x = position.x;
        pose.position.y = position.y;
        pose.position.z = position.z;
        tf2::Quaternion q;
        q.setRPY(orientation.R,orientation.P,orientation.Y);
        //Set orientation components
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }
    
    void sendStartRequest() override {
        client_ptr->waitForServer();
        bt_drs_msgs::manipulateTargetGoal goal;

        auto position_bt = getInput<Position3D>("position");
        auto orientation_bt = getInput<Orientation3D>("orientation_3D");
        auto device_type = getInput<std::string>("device_type");
        auto strategy = getInput<int>("strategy");
        auto direction = getInput<int>("direction");
        auto manipulation_todo = getInput<double>("manipulation_todo");

        if (!position_bt && !orientation_bt) {
            throw BT::RuntimeError("Missing required inputs");
        }
    
        Position3D position;
        Orientation3D orientation;

        position.x = position_bt.value().x;
        position.y = position_bt.value().y;
        position.z = position_bt.value().z;

        orientation.R = orientation_bt.value().R;
        orientation.P = orientation_bt.value().P;
        orientation.Y = orientation_bt.value().Y;

        goal.target = GetPoseFromEuler(position,orientation);
        goal.device_type = device_type.value();
        goal.strategy = strategy.value();
        goal.direction = direction.value();
        goal.manipulation_todo = manipulation_todo.value();
        std::cout << "Sending manipulation goal..." << std::endl;

        client_ptr->sendGoal(goal);
    }

    bool getResult() override {
        bt_drs_msgs::manipulateTargetResultConstPtr result;
        result = client_ptr->getResult();
        return result->result;
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

