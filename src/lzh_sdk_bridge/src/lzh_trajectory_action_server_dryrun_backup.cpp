#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class G1DTrajectoryActionServer : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJT = rclcpp_action::ServerGoalHandle<FollowJT>;

  G1DTrajectoryActionServer()
  : Node("lzh_trajectory_action_server_dryrun_backup")
  {
    dry_run_ = this->declare_parameter<bool>("dry_run", true);

    left_joints_ = {
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      "left_wrist_pitch_joint",
      "left_wrist_yaw_joint"
    };

    right_joints_ = {
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint",
      "right_wrist_pitch_joint",
      "right_wrist_yaw_joint"
    };

    left_server_ = rclcpp_action::create_server<FollowJT>(
      this,
      "/left_arm_controller/follow_joint_trajectory",
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJT::Goal> goal)
      {
        return this->handle_goal(uuid, goal, "left_arm", left_joints_);
      },
      [this](const std::shared_ptr<GoalHandleFollowJT> goal_handle)
      {
        return this->handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleFollowJT> goal_handle)
      {
        this->handle_accepted(goal_handle, "left_arm", left_joints_);
      });

    right_server_ = rclcpp_action::create_server<FollowJT>(
      this,
      "/right_arm_controller/follow_joint_trajectory",
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJT::Goal> goal)
      {
        return this->handle_goal(uuid, goal, "right_arm", right_joints_);
      },
      [this](const std::shared_ptr<GoalHandleFollowJT> goal_handle)
      {
        return this->handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleFollowJT> goal_handle)
      {
        this->handle_accepted(goal_handle, "right_arm", right_joints_);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "G1-D trajectory action server started. dry_run=%s",
      dry_run_ ? "true" : "false");

    RCLCPP_INFO(
      this->get_logger(),
      "Left action : /left_arm_controller/follow_joint_trajectory");

    RCLCPP_INFO(
      this->get_logger(),
      "Right action: /right_arm_controller/follow_joint_trajectory");
  }

private:
  bool dry_run_;

  std::vector<std::string> left_joints_;
  std::vector<std::string> right_joints_;

  rclcpp_action::Server<FollowJT>::SharedPtr left_server_;
  rclcpp_action::Server<FollowJT>::SharedPtr right_server_;

  static int64_t duration_to_ns(const builtin_interfaces::msg::Duration & d)
  {
    return static_cast<int64_t>(d.sec) * 1000000000LL +
           static_cast<int64_t>(d.nanosec);
  }

  static bool is_finite_vector(const std::vector<double> & values)
  {
    return std::all_of(
      values.begin(),
      values.end(),
      [](double value)
      {
        return std::isfinite(value);
      });
  }

  bool validate_trajectory(
    const FollowJT::Goal & goal,
    const std::vector<std::string> & expected_joints,
    std::string & error_message) const
  {
    const auto & trajectory = goal.trajectory;

    if (trajectory.points.empty()) {
      error_message = "Trajectory contains no points.";
      return false;
    }

    if (trajectory.joint_names.size() != expected_joints.size()) {
      std::ostringstream oss;
      oss << "Expected " << expected_joints.size()
          << " joints, but received "
          << trajectory.joint_names.size() << ".";
      error_message = oss.str();
      return false;
    }

    std::unordered_set<std::string> received_set;

    for (const auto & name : trajectory.joint_names) {
      if (!received_set.insert(name).second) {
        error_message = "Duplicate joint name: " + name;
        return false;
      }
    }

    std::unordered_set<std::string> expected_set(
      expected_joints.begin(),
      expected_joints.end());

    for (const auto & name : trajectory.joint_names) {
      if (expected_set.find(name) == expected_set.end()) {
        error_message = "Unexpected joint name: " + name;
        return false;
      }
    }

    for (const auto & expected : expected_joints) {
      if (received_set.find(expected) == received_set.end()) {
        error_message = "Missing joint: " + expected;
        return false;
      }
    }

    int64_t previous_time_ns = -1;

    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & point = trajectory.points[i];

      if (point.positions.size() != trajectory.joint_names.size()) {
        std::ostringstream oss;
        oss << "Point " << i
            << " has positions size "
            << point.positions.size()
            << ", expected "
            << trajectory.joint_names.size() << ".";
        error_message = oss.str();
        return false;
      }

      if (!point.velocities.empty() &&
          point.velocities.size() != trajectory.joint_names.size())
      {
        std::ostringstream oss;
        oss << "Point " << i
            << " has velocities size "
            << point.velocities.size()
            << ", expected "
            << trajectory.joint_names.size() << ".";
        error_message = oss.str();
        return false;
      }

      if (!is_finite_vector(point.positions)) {
        error_message =
          "Point " + std::to_string(i) +
          " contains NaN or Inf in positions.";
        return false;
      }

      if (!point.velocities.empty() &&
          !is_finite_vector(point.velocities))
      {
        error_message =
          "Point " + std::to_string(i) +
          " contains NaN or Inf in velocities.";
        return false;
      }

      const int64_t current_time_ns =
        duration_to_ns(point.time_from_start);

      if (current_time_ns < 0) {
        error_message =
          "Point " + std::to_string(i) +
          " has negative time_from_start.";
        return false;
      }

      if (i > 0 && current_time_ns <= previous_time_ns) {
        error_message =
          "time_from_start is not strictly increasing at point " +
          std::to_string(i) + ".";
        return false;
      }

      previous_time_ns = current_time_ns;
    }

    return true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJT::Goal> goal,
    const std::string & arm_name,
    const std::vector<std::string> & expected_joints)
  {
    if (!dry_run_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[%s] Real SDK2 execution is not implemented yet. Rejecting goal.",
        arm_name.c_str());

      return rclcpp_action::GoalResponse::REJECT;
    }

    std::string error_message;

    if (!validate_trajectory(
        *goal,
        expected_joints,
        error_message))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "[%s] Rejecting invalid trajectory: %s",
        arm_name.c_str(),
        error_message.c_str());

      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Valid trajectory goal accepted.",
      arm_name.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJT>)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Cancel request received.");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandleFollowJT> goal_handle,
    const std::string & arm_name,
    const std::vector<std::string> & expected_joints)
  {
    std::thread(
      [this, goal_handle, arm_name, expected_joints]()
      {
        this->execute(
          goal_handle,
          arm_name,
          expected_joints);
      }).detach();
  }

  std::vector<double> reorder_positions(
    const std::vector<std::string> & incoming_names,
    const std::vector<double> & incoming_positions,
    const std::vector<std::string> & expected_joints) const
  {
    std::unordered_map<std::string, std::size_t> index_map;

    for (std::size_t i = 0; i < incoming_names.size(); ++i) {
      index_map[incoming_names[i]] = i;
    }

    std::vector<double> ordered;
    ordered.reserve(expected_joints.size());

    for (const auto & joint : expected_joints) {
      ordered.push_back(
        incoming_positions[index_map.at(joint)]);
    }

    return ordered;
  }

  void print_positions(
    const std::string & label,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "%s",
      label.c_str());

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(
        this->get_logger(),
        "  %-32s %.6f rad",
        joint_names[i].c_str(),
        positions[i]);
    }
  }

  void execute(
    const std::shared_ptr<GoalHandleFollowJT> goal_handle,
    const std::string arm_name,
    const std::vector<std::string> expected_joints)
  {
    const auto goal = goal_handle->get_goal();
    const auto & trajectory = goal->trajectory;

    if (goal_handle->is_canceling()) {
      auto result =
        std::make_shared<FollowJT::Result>();

      result->error_code =
        FollowJT::Result::SUCCESSFUL;

      result->error_string =
        "Dry-run goal canceled.";

      goal_handle->canceled(result);
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "========== DRY RUN: %s ==========",
      arm_name.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory points: %zu",
      trajectory.points.size());

    RCLCPP_INFO(
      this->get_logger(),
      "Incoming joint order:");

    for (std::size_t i = 0;
         i < trajectory.joint_names.size();
         ++i)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "  [%zu] %s",
        i,
        trajectory.joint_names[i].c_str());
    }

    const auto first_positions =
      reorder_positions(
        trajectory.joint_names,
        trajectory.points.front().positions,
        expected_joints);

    const auto last_positions =
      reorder_positions(
        trajectory.joint_names,
        trajectory.points.back().positions,
        expected_joints);

    print_positions(
      "First point (canonical joint order):",
      expected_joints,
      first_positions);

    print_positions(
      "Last point (canonical joint order):",
      expected_joints,
      last_positions);

    const auto & last_time =
      trajectory.points.back().time_from_start;

    RCLCPP_INFO(
      this->get_logger(),
      "Final time_from_start: %d.%09u s",
      last_time.sec,
      last_time.nanosec);

    RCLCPP_INFO(
      this->get_logger(),
      "DRY RUN ONLY: no Unitree SDK2 LowCmd was sent.");

    auto result =
      std::make_shared<FollowJT::Result>();

    result->error_code =
      FollowJT::Result::SUCCESSFUL;

    result->error_string =
      "Dry-run trajectory validation successful.";

    goal_handle->succeed(result);

    RCLCPP_INFO(
      this->get_logger(),
      "========== DRY RUN COMPLETE ==========");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<G1DTrajectoryActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
