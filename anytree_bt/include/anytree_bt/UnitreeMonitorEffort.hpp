#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <mutex>
#include <ostream>

struct UnitreeJointEfforts {
  double j1_effort{}, j2_effort{}, j3_effort{}, j4_effort{}, j5_effort{}, j6_effort{};
};
bool operator<(const UnitreeJointEfforts &x, const UnitreeJointEfforts &y) {
  return (x.j1_effort < y.j1_effort &&
          x.j2_effort < y.j2_effort &&
          x.j3_effort < y.j3_effort &&
          x.j4_effort < y.j4_effort &&
          x.j5_effort < y.j5_effort &&
          x.j6_effort < y.j6_effort);
}
std::ostream &operator<<(std::ostream &os, const UnitreeJointEfforts &efforts) {
  os << "["
      << efforts.j1_effort << ", "
      << efforts.j2_effort << ", "
      << efforts.j3_effort << ", "
      << efforts.j4_effort << ", "
      << efforts.j5_effort << ", "
      << efforts.j6_effort << "]";
  return os;
}

namespace BT {
template <> inline UnitreeJointEfforts convertFromString(StringView str) {
  // real numbers separated by space
  auto parts = splitString(str, ' ');
  if (parts.size() != 6) {
    throw RuntimeError("invalid input)");
  } else {
    UnitreeJointEfforts output;
    output.j1_effort = convertFromString<double>(parts[0]);
    output.j2_effort = convertFromString<double>(parts[1]);
    output.j3_effort = convertFromString<double>(parts[2]);
    output.j4_effort = convertFromString<double>(parts[3]);
    output.j5_effort = convertFromString<double>(parts[4]);
    output.j6_effort = convertFromString<double>(parts[5]);
    return output;
  }
}
} // namespace BT

class UnitreeMonitorEffort : public BT::ConditionNode {
public:
  UnitreeMonitorEffort(const std::string &name,
                              const BT::NodeConfig &config, ros::NodeHandle nh)
      : BT::ConditionNode(name, config), nh_(nh) {
    sub_ = nh_.subscribe(
        "z1_gazebo/joint_states_filtered", 10, &UnitreeMonitorEffort::jointStateCallback, this);
  }

  virtual ~UnitreeMonitorEffort() override = default;

  static BT::PortsList providedPorts() {
    return {BT::InputPort<UnitreeJointEfforts>("threshold")};
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::mutex mutex_;
  UnitreeJointEfforts last_effort_measurement_;

  virtual BT::NodeStatus tick() override {
    auto threshold = getInput<UnitreeJointEfforts>("threshold");
    if (!threshold) {
      throw BT::RuntimeError(
          "Missing parameter [threshold] in UnitreeMonitorEffort");
    }

    mutex_.lock();
    bool is_ft_below_threshold = last_effort_measurement_ < threshold.value();
    mutex_.unlock();

    if (is_ft_below_threshold) {
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR_STREAM("[UnitreeMonitorEffort] Threshold exceeded: "
                       << last_effort_measurement_ << " > " << threshold.value());
      return BT::NodeStatus::FAILURE;
    }
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr &joint_states) {
    mutex_.lock();
    last_effort_measurement_.j1_effort = std::abs(joint_states->effort[0]);
    last_effort_measurement_.j2_effort = std::abs(joint_states->effort[1]);
    last_effort_measurement_.j3_effort = std::abs(joint_states->effort[2]);
    last_effort_measurement_.j4_effort = std::abs(joint_states->effort[3]);
    last_effort_measurement_.j5_effort = std::abs(joint_states->effort[4]);
    last_effort_measurement_.j6_effort = std::abs(joint_states->effort[5]);
    // ROS_INFO_STREAM("New F/T measurement received: " <<
    // last_ft_measurement_);
    mutex_.unlock();
  }
};
