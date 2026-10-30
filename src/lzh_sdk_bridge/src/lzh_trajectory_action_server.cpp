#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <stdexcept>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_environment_guard.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;
using namespace unitree::robot;

class G1DTrajectoryActionServer : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GripperAction = control_msgs::action::GripperCommand;
  using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJT>;
  static constexpr std::size_t N = 35;
  static constexpr std::size_t BODY_MOTOR_COUNT = 29;

  G1DTrajectoryActionServer() : Node("lzh_trajectory_action_server")
  {
    dry_run_ = declare_parameter<bool>("dry_run", true);
    enable_real_execution_ =
      declare_parameter<bool>("enable_real_execution", false);
    // Explicit opt-in for a single, coordinated 14-joint trajectory.
    // RViz/MoveIt both_arms PLAN remains available when disabled.
    declare_parameter<bool>("enable_both_arms_real_execution", false);
    // Permit Dex1 left/right during an active coordinated both_arms Action.
    // Keep disabled until the combined motion has been reviewed in MoveIt.
    declare_parameter<bool>("enable_quad_real_execution", false);
    network_interface_ =
      declare_parameter<std::string>("network_interface", "eno2");
    control_period_ms_ =
      declare_parameter<double>("control_period_ms", 5.0);

    // Arm commands go over rt/arm_sdk, the channel the robot blends with its
    // own gravity-compensated arm controller (weight in motor_cmd[29].q).
    // rt/lowcmd bypasses that controller: with tau=0 the arms sag by
    // gravity/kp and every joint this node touches loses its support.
    arm_sdk_topic_ = declare_parameter<std::string>("arm_sdk_topic", "rt/arm_sdk");
    arm_sdk_weight_rate_ =
      declare_parameter<double>("arm_sdk_weight_rate", 0.5);  // 1/0 in ~2 s

    // Independent Dex1-1 commissioning gate: disabled by default.
    // Enabling this does NOT enable either arm FollowJointTrajectory server.
    enable_gripper_real_execution_ =
      declare_parameter<bool>("enable_gripper_real_execution", false);
    gripper_max_opening_m_ =
      declare_parameter<double>("gripper_max_opening_m", 0.09);
    gripper_speed_limit_rad_s_ =
      declare_parameter<double>("gripper_speed_limit_rad_s", 0.5);
    gripper_goal_tolerance_rad_ =
      declare_parameter<double>("gripper_goal_tolerance_rad", 0.03);
    gripper_goal_timeout_sec_ =
      declare_parameter<double>("gripper_goal_timeout_sec", 4.0);
    gripper_feedback_timeout_sec_ =
      declare_parameter<double>("gripper_feedback_timeout_sec", 0.25);

    // Temporary commissioning guardrails; NOT official mechanical limits.
    max_start_error_rad_ =
      declare_parameter<double>("max_start_error_rad", 0.03);
    // Independent conservative execution-speed guard. Unlike position limits,
    // this is a commissioning speed bound, NOT Unitree's motor velocity rating.
    arm_max_trajectory_speed_rad_s_ =
      declare_parameter<double>("arm_max_trajectory_speed_rad_s", 0.5);
    lowstate_timeout_sec_ =
      declare_parameter<double>("lowstate_timeout_sec", 0.20);

    // Wait for the measured real joints to settle before reporting SUCCESS.
    goal_tolerance_rad_ =
      declare_parameter<double>("goal_tolerance_rad", 0.03);
    settle_duration_sec_ =
      declare_parameter<double>("settle_duration_sec", 0.20);
    settle_timeout_sec_ =
      declare_parameter<double>("settle_timeout_sec", 2.0);


    left_joints_ = {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint", "left_elbow_joint",
      "left_wrist_roll_joint", "left_wrist_pitch_joint",
      "left_wrist_yaw_joint"
    };
    right_joints_ = {
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint", "right_elbow_joint",
      "right_wrist_roll_joint", "right_wrist_pitch_joint",
      "right_wrist_yaw_joint"
    };
    left_ids_  = {15, 16, 17, 18, 19, 20, 21};
    right_ids_ = {22, 23, 24, 25, 26, 27, 28};
    both_joints_ = left_joints_;
    both_joints_.insert(both_joints_.end(), right_joints_.begin(), right_joints_.end());
    both_ids_ = left_ids_;
    both_ids_.insert(both_ids_.end(), right_ids_.begin(), right_ids_.end());

    left_server_ = rclcpp_action::create_server<FollowJT>(
      this, "/left_arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID & id,
             std::shared_ptr<const FollowJT::Goal> goal) {
        return handle_goal(id, goal, "left_arm", left_joints_);
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        return handle_cancel(gh);
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        handle_accepted(gh, "left_arm", left_joints_, left_ids_);
      });

    right_server_ = rclcpp_action::create_server<FollowJT>(
      this, "/right_arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID & id,
             std::shared_ptr<const FollowJT::Goal> goal) {
        return handle_goal(id, goal, "right_arm", right_joints_);
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        return handle_cancel(gh);
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        handle_accepted(gh, "right_arm", right_joints_, right_ids_);
      });

    // Coordinated both-arm controller: one 14-joint trajectory, one time base,
    // one SDK2 LowCmd writer. This does NOT run two uncoordinated arm Actions.
    // Disabled for real motion until explicitly enabled by an operator.
    both_server_ = rclcpp_action::create_server<FollowJT>(
      this, "/both_arms_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID & id,
             std::shared_ptr<const FollowJT::Goal> goal) {
        return handle_goal(id, goal, "both_arms", both_joints_);
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        return handle_cancel(gh);
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        handle_accepted(gh, "both_arms", both_joints_, both_ids_);
      });

    // Two independent GripperCommand interfaces, multiplexed into one LowCmd
    // writer. No extra DDS publisher is created.
    left_gripper_server_ =
      rclcpp_action::create_server<GripperAction>(
        this, "/lzh_left_dex1/gripper_command",
        [this](const rclcpp_action::GoalUUID & id,
               std::shared_ptr<const GripperAction::Goal> goal) {
          return handle_gripper_goal(id, goal, 0);
        },
        [this](const std::shared_ptr<GripperGoalHandle> gh) {
          return handle_gripper_cancel(gh);
        },
        [this](const std::shared_ptr<GripperGoalHandle> gh) {
          handle_gripper_accepted(gh, 0);
        });

    right_gripper_server_ =
      rclcpp_action::create_server<GripperAction>(
        this, "/lzh_right_dex1/gripper_command",
        [this](const rclcpp_action::GoalUUID & id,
               std::shared_ptr<const GripperAction::Goal> goal) {
          return handle_gripper_goal(id, goal, 1);
        },
        [this](const std::shared_ptr<GripperGoalHandle> gh) {
          return handle_gripper_cancel(gh);
        },
        [this](const std::shared_ptr<GripperGoalHandle> gh) {
          handle_gripper_accepted(gh, 1);
        });

    // Subscribe to validated raw Dex1 motor feedback.
    // No gripper motor commands are sent here.
    dex1_feedback_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/lzh_dex1/motor_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(dex1_mutex_);

          // The bridge omits missing/invalid motors from this message.
          dex1_online_ = {false, false};
          const auto now = std::chrono::steady_clock::now();

          const std::array<std::string, 2> names = {
            "left_dex1_motor_raw",
            "right_dex1_motor_raw"
          };

          for (std::size_t i = 0; i < msg->name.size(); ++i) {
            if (i >= msg->position.size()) {
              break;
            }

            for (std::size_t side = 0; side < 2; ++side) {
              if (msg->name[i] == names[side] &&
                  std::isfinite(msg->position[i])) {
                dex1_q_[side] = msg->position[i];
                dex1_online_[side] = true;
                dex1_last_feedback_[side] = now;
              }
            }
          }

        });

    // Read-only diagnostic publisher: motor command targets already computed
    // by this ActionServer. This does not modify or transmit additional LowCmd.
    arm_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/g1d/arm_commanded_joint_states", 10);
    arm_command_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publish_arm_command_snapshot(); });

    RCLCPP_INFO(get_logger(),
      "started: dry_run=%s arm_real=%s gripper_real=%s",
      dry_run_ ? "true" : "false",
      enable_real_execution_ ? "true" : "false",
      enable_gripper_real_execution_ ? "true" : "false");

    if (!dry_run_ && (enable_real_execution_ || enable_gripper_real_execution_)) {
      init_sdk2();
    } else {
      RCLCPP_WARN(get_logger(), "REAL LowCmd output is disabled.");
    }
  }

  ~G1DTrajectoryActionServer() override
  {
    stop_writer_.store(true);
    if (writer_.joinable()) writer_.join();
    std::lock_guard<std::mutex> lk(gripper_threads_mtx_);
    for (auto & thread : gripper_threads_) {
      if (thread.joinable()) thread.join();
    }
  }

private:
  bool dry_run_{true};
  bool enable_real_execution_{false};
  bool enable_gripper_real_execution_{false};
  double gripper_max_opening_m_{0.09};

  double gripper_speed_limit_rad_s_{0.5};
  double gripper_goal_tolerance_rad_{0.03};
  double gripper_goal_timeout_sec_{4.0};
  double gripper_feedback_timeout_sec_{0.25};
  // Unitree xr_teleoperate Dex1-1: nominal complete travel = 0..5.4 rad
  // (0..90 mm); 0.18 rad is a per-control-update feedback-relative cap,
  // not the maximum angular displacement of an entire Action goal.
  static constexpr double GRIPPER_MOTOR_STROKE_RAD = 5.4;
  static constexpr double GRIPPER_OPENING_STROKE_M = 0.09;
  static constexpr double GRIPPER_OFFICIAL_CMD_DELTA_RAD = 0.18;

  // Unitree's g1_d_description/g1_d.urdf, 7 joints per arm, in joint order.
  // Official joint-position bounds do not replace MoveIt collision checking.
  static constexpr std::array<double, 7> LEFT_ARM_MIN{{
    -3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558
  }};
  static constexpr std::array<double, 7> LEFT_ARM_MAX{{
     2.6704,  2.2515,  2.618,  2.0944,  1.972222054,  1.614429558,  1.614429558
  }};
  static constexpr std::array<double, 7> RIGHT_ARM_MIN{{
    -3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558
  }};
  static constexpr std::array<double, 7> RIGHT_ARM_MAX{{
     2.6704,  1.5882,  2.618,  2.0944,  1.972222054,  1.614429558,  1.614429558
  }};
  static constexpr float GRIPPER_KP = 5.0f;
  static constexpr float GRIPPER_KD = 0.05f;
  static constexpr std::array<std::size_t, 2> GRIPPER_MOTOR_IDS{{31, 33}};
  std::string network_interface_{"eno2"};
  double control_period_ms_{5.0};
  // rt/arm_sdk slots: motor_cmd[29].q() is the SDK weight (0 = the robot's own
  // arm controller, 1 = this node). Official G1 examples write it this way.
  static constexpr std::size_t ARM_SDK_WEIGHT_SLOT = 29;
  std::string arm_sdk_topic_{"rt/arm_sdk"};
  double arm_sdk_weight_rate_{0.5};
  std::atomic<float> arm_sdk_weight_{0.0f};
  std::atomic<float> arm_sdk_weight_target_{0.0f};
  double max_start_error_rad_{0.03};
  double arm_max_trajectory_speed_rad_s_{0.5};
  double lowstate_timeout_sec_{0.20};
  double goal_tolerance_rad_{0.03};
  double settle_duration_sec_{0.20};
  double settle_timeout_sec_{2.0};

  std::vector<std::string> left_joints_, right_joints_, both_joints_;
  std::vector<int> left_ids_, right_ids_, both_ids_;
  rclcpp_action::Server<FollowJT>::SharedPtr left_server_, right_server_, both_server_;

  rclcpp_action::Server<GripperAction>::SharedPtr
    left_gripper_server_;

  rclcpp_action::Server<GripperAction>::SharedPtr
    right_gripper_server_;

  // Raw gripper feedback for future control safety checks.
  std::mutex dex1_mutex_;
  std::array<bool, 2> dex1_online_{{false, false}};
  std::array<double, 2> dex1_q_{{0.0, 0.0}};
  std::array<std::chrono::steady_clock::time_point, 2> dex1_last_feedback_{};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    dex1_feedback_sub_;

  ChannelPublisherPtr<LowCmd_> lowcmd_pub_;
  ChannelSubscriberPtr<LowState_> lowstate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::TimerBase::SharedPtr arm_command_timer_;

  std::mutex state_mtx_, cmd_mtx_;
  std::array<double, N> latest_q_{};
  std::array<double, N> desired_q_{};
  uint8_t mode_machine_{0};
  bool have_lowstate_{false};
  bool command_enabled_{false};
  bool gripper_fault_latched_{false};  // Cleared by node restart, never auto-reset.
  std::array<bool, 2> gripper_control_initialized_{{false, false}};
  std::mutex session_mtx_;
  std::array<bool, 2> gripper_goal_active_{{false, false}};
  std::mutex gripper_threads_mtx_;
  std::array<std::thread, 2> gripper_threads_;
  std::chrono::steady_clock::time_point last_lowstate_;

  std::atomic<bool> stop_writer_{false};
  std::atomic<bool> real_goal_active_{false};
  std::atomic<bool> both_arm_goal_active_{false};
  std::thread writer_;

  // Official G1 arm gains (unitree_lerobot robot_arm.py): shoulder/elbow 80/3,
  // wrist 40/1.5. The bring-up example's 40/1 leaves a steady-state error of
  // gravity/kp (~0.09 rad at the elbow); 80 halves it and the arm_sdk weight
  // ramp removes the takeover step. Residual droop still has to be measured
  // on the bench: tau stays 0, so this is a PD hold either way.
  const std::array<float, N> kp_{
    0,0,0,0,0,0, 0,0,0,0,0,0, 60,0,40,
    80,80,80,80, 40,40,40,
    80,80,80,80, 40,40,40,
    0,0,0,0,0,0
  };
  const std::array<float, N> kd_{
    0,0,0,0,0,0, 0,0,0,0,0,0, 1,0,1,
    3,3,3,3, 1.5,1.5,1.5,
    3,3,3,3, 1.5,1.5,1.5,
    0,0,0,0,0,0
  };

  static uint32_t crc32_core(uint32_t * ptr, uint32_t len)
  {
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0x04c11db7;
    for (uint32_t i = 0; i < len; ++i) {
      uint32_t xbit = 1U << 31;
      uint32_t data = ptr[i];
      for (uint32_t b = 0; b < 32; ++b) {
        if (crc & 0x80000000) { crc <<= 1; crc ^= poly; }
        else crc <<= 1;
        if (data & xbit) crc ^= poly;
        xbit >>= 1;
      }
    }
    return crc;
  }

  void init_sdk2()
  {
    RCLCPP_WARN(get_logger(),
      "Initializing REAL SDK2 on %s. No LowCmd is sent until a valid goal is accepted.",
      network_interface_.c_str());

    ChannelFactory::Instance()->Init(0, network_interface_);

    lowcmd_pub_.reset(new ChannelPublisher<LowCmd_>(arm_sdk_topic_));
    lowcmd_pub_->InitChannel();

    lowstate_sub_.reset(new ChannelSubscriber<LowState_>("rt/lowstate"));
    lowstate_sub_->InitChannel(
      [this](const void * msg) { lowstate_cb(msg); }, 1);

    writer_ = std::thread([this]() { writer_loop(); });
  }

  void lowstate_cb(const void * msg)
  {
    LowState_ s = *static_cast<const LowState_ *>(msg);
    auto * raw = reinterpret_cast<uint32_t *>(&s);
    if (s.crc() != crc32_core(raw, (sizeof(LowState_) >> 2) - 1)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "LowState CRC error; frame ignored.");
      return;
    }

    std::lock_guard<std::mutex> lk(state_mtx_);
    for (std::size_t i = 0; i < N; ++i) {
      latest_q_[i] = s.motor_state().at(i).q();
    }
    mode_machine_ = s.mode_machine();
    have_lowstate_ = true;
    last_lowstate_ = std::chrono::steady_clock::now();
  }

  bool latest_state(std::array<double, N> & q, uint8_t & mode, double & age)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!have_lowstate_) return false;
    q = latest_q_;
    mode = mode_machine_;
    age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_lowstate_).count();
    return true;
  }

  // Read-only monitoring at 2 Hz, independent of the SDK2 LowCmd writer.
  // The published positions are *command targets*, not measurements.
  // Compare to the existing /joint_states topic with matching names.
  void publish_arm_command_snapshot()
  {
    std::array<double, N> command{};
    bool enabled = false;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      enabled = command_enabled_;
      command = desired_q_;
    }
    if (!enabled || !arm_command_pub_) return;

    std::array<double, N> actual{};
    uint8_t mode = 0;
    double age = 0.0;
    if (!latest_state(actual, mode, age) || age > lowstate_timeout_sec_) return;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (std::size_t i = 0; i < left_ids_.size(); ++i) {
      msg.name.push_back(left_joints_[i]);
      msg.position.push_back(command.at(static_cast<std::size_t>(left_ids_[i])));
    }
    for (std::size_t i = 0; i < right_ids_.size(); ++i) {
      msg.name.push_back(right_joints_[i]);
      msg.position.push_back(command.at(static_cast<std::size_t>(right_ids_[i])));
    }
    arm_command_pub_->publish(msg);

    constexpr std::size_t right_elbow_id = 25;
    const double elbow_error = command[right_elbow_id] - actual[right_elbow_id];
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "RIGHT_ELBOW_TRACKING q_cmd=%.6f q_actual=%.6f error=%.6f rad "
      "kp=%.1f kd=%.1f command_enabled=true",
      command[right_elbow_id], actual[right_elbow_id], elbow_error,
      static_cast<double>(kp_[right_elbow_id]),
      static_cast<double>(kd_[right_elbow_id]));
  }

  void writer_loop()
  {
    const auto period = std::chrono::duration<double, std::milli>(control_period_ms_);

    while (!stop_writer_.load()) {
      const auto t0 = std::chrono::steady_clock::now();

      bool enabled;
      std::array<double, N> qcmd{};
      std::array<bool, 2> gripper_enabled{};
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        enabled = command_enabled_;
        qcmd = desired_q_;
        gripper_enabled = gripper_control_initialized_;
      }

      // Ramp the arm_sdk weight toward its target: 1 while a goal owns the
      // arms, 0 after it ends. Official examples blend in and out this way so
      // control never changes hands abruptly.
      const float weight_target = arm_sdk_weight_target_.load();
      float weight = arm_sdk_weight_.load();
      const float weight_step =
        static_cast<float>(arm_sdk_weight_rate_ * (control_period_ms_ / 1000.0));
      weight = (weight < weight_target)
        ? std::min(weight_target, weight + weight_step)
        : std::max(weight_target, weight - weight_step);
      arm_sdk_weight_.store(weight);

      // Keep publishing while the weight ramps down, so the blend returns to
      // the robot's own controller instead of stopping mid-handover.
      if ((enabled || weight > 0.0f) && lowcmd_pub_) {
        std::array<double, N> unused{};
        uint8_t mode = 0;
        double age = 0.0;

        if (latest_state(unused, mode, age) && age <= lowstate_timeout_sec_) {
          if (!valid_body_hold_state(unused) ||
              ((gripper_enabled[0] || gripper_enabled[1]) &&
               !real_goal_active_.load() &&
               std::abs(qcmd[21] - unused[21]) > 0.15)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
              "Body feedback invalid or left wrist jumped: LowCmd stopped");
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            command_enabled_ = false;
            gripper_fault_latched_ = true;
            continue;
          }
          LowCmd_ cmd;
          cmd.mode_pr() = static_cast<uint8_t>(0);
          cmd.mode_machine() = mode;

          bool all_finite = true;
          for (std::size_t i = 0; i < BODY_MOTOR_COUNT; ++i) {
            all_finite = all_finite && std::isfinite(qcmd[i]);
          }
          for (std::size_t side = 0; side < 2; ++side) {
            if (gripper_enabled[side]) {
              const auto id = GRIPPER_MOTOR_IDS[side];
              all_finite = all_finite &&
                std::isfinite(qcmd[id]) && std::isfinite(unused[id]) &&
                unused[id] >= -0.03 &&
                unused[id] <= GRIPPER_MOTOR_STROKE_RAD + 0.05;
            }
          }
          if (!all_finite) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
              "Non-finite LowCmd detected; no command sent.");
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            command_enabled_ = false;
            gripper_fault_latched_ = true;
            continue;
          }

          for (std::size_t i = 0; i < BODY_MOTOR_COUNT; ++i) {
            cmd.motor_cmd().at(i).mode() = 1;
            cmd.motor_cmd().at(i).tau() = 0.0F;
            cmd.motor_cmd().at(i).q() = static_cast<float>(qcmd[i]);
            cmd.motor_cmd().at(i).dq() = 0.0F;
            cmd.motor_cmd().at(i).kp() = kp_[i];
            cmd.motor_cmd().at(i).kd() = kd_[i];
          }

          // The message always contains 35 motor slots. Explicitly leave
          // non-body, non-gripper slots disabled rather than using body gains.
          // Slot 29 carries the rt/arm_sdk weight, not a motor command.
          for (std::size_t i = BODY_MOTOR_COUNT; i < N; ++i) {
            auto & motor = cmd.motor_cmd().at(i);
            motor.mode() = 0;
            motor.tau() = 0.0F;
            motor.q() = (i == ARM_SDK_WEIGHT_SLOT)
              ? std::clamp(weight, 0.0f, 1.0f)
              : static_cast<float>(unused[i]);
            motor.dq() = 0.0F;
            motor.kp() = 0.0F;
            motor.kd() = 0.0F;
          }
          for (std::size_t side = 0; side < 2; ++side) {
            if (!gripper_enabled[side]) continue;
            const std::size_t id = GRIPPER_MOTOR_IDS[side];
            auto & motor = cmd.motor_cmd().at(id);
            motor.mode() = 1;
            motor.tau() = 0.0F;
            // Official XR controller clips each outgoing command relative
            // to the current motor feedback by +/- 0.18 rad. Keep the same
            // per-update bound, even for a full-stroke Action target.
            const double lo = std::max(
              0.0, unused[id] - GRIPPER_OFFICIAL_CMD_DELTA_RAD);
            const double hi = std::min(
              GRIPPER_MOTOR_STROKE_RAD,
              unused[id] + GRIPPER_OFFICIAL_CMD_DELTA_RAD);
            motor.q() = static_cast<float>(std::clamp(qcmd[id], lo, hi));
            motor.dq() = 0.0F;
            motor.kp() = GRIPPER_KP;
            motor.kd() = GRIPPER_KD;
          }

          cmd.crc() = crc32_core(
            reinterpret_cast<uint32_t *>(&cmd),
            (sizeof(cmd) >> 2) - 1);
          lowcmd_pub_->Write(cmd);
        } else {
          {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            command_enabled_ = false;  // No automatic resumption on recovery.
            if (gripper_enabled[0] || gripper_enabled[1]) {
              gripper_fault_latched_ = true;
            }
          }
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "LowState missing/stale; LowCmd stopped (no auto-resume).");
        }
      }

      std::this_thread::sleep_until(
        t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }
  }

  // ROS GripperCommand.position is the total opening (meters), whereas
  // Dex1's internal motor takes radians. This calibration is preliminary;
  // the defaults match Unitree's nominal 5.4 rad / 90 mm reference.
  static double opening_to_motor(double opening_m)
  {
    return opening_m * (GRIPPER_MOTOR_STROKE_RAD / GRIPPER_OPENING_STROKE_M);
  }

  static double motor_to_opening(double motor_q)
  {
    return motor_q * (GRIPPER_OPENING_STROKE_M / GRIPPER_MOTOR_STROKE_RAD);
  }

  bool gripper_feedback(std::size_t side, double & q, double & age)
  {
    std::lock_guard<std::mutex> lk(dex1_mutex_);
    if (!dex1_online_[side]) return false;
    age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - dex1_last_feedback_[side]).count();
    q = dex1_q_[side];
    return std::isfinite(q) && age <= gripper_feedback_timeout_sec_;
  }

  bool valid_body_hold_state(const std::array<double, N> & q)
  {
    for (std::size_t i = 0; i < BODY_MOTOR_COUNT; ++i) {
      if (!std::isfinite(q[i])) return false;
    }
    // Check all real arm joints against official G1-D position limits.
    // Allow only a small measurement/calibration tolerance on raw feedback;
    // commanded target positions are checked against exact limits below.
    for (std::size_t j = 0; j < 7; ++j) {
      const double l = q[static_cast<std::size_t>(left_ids_[j])];
      const double r = q[static_cast<std::size_t>(right_ids_[j])];
      if (l < LEFT_ARM_MIN[j] - 0.03 || l > LEFT_ARM_MAX[j] + 0.03 ||
          r < RIGHT_ARM_MIN[j] - 0.03 || r > RIGHT_ARM_MAX[j] + 0.03) {
        return false;
      }
    }
    return true;
  }

  rclcpp_action::GoalResponse handle_gripper_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const GripperAction::Goal> goal,
    std::size_t side)
  {
    const double opening = goal->command.position;
    const double max_effort = goal->command.max_effort;
    if (!std::isfinite(opening) || opening < 0.0 ||
        opening > std::min(gripper_max_opening_m_, GRIPPER_OPENING_STROKE_M) ||
        !std::isfinite(max_effort) || max_effort != 0.0) {
      RCLCPP_ERROR(get_logger(),
        "Dex1 goal rejected: position must be 0..%.3f m; "
        "nonzero max_effort unsupported without force calibration",
        gripper_max_opening_m_);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (dry_run_) return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    if (!enable_gripper_real_execution_) {
      RCLCPP_ERROR(get_logger(),
        "Dex1 real output disabled: enable_gripper_real_execution=false");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lk(session_mtx_);
    const bool quad_arm_allowed =
      real_goal_active_.load() && both_arm_goal_active_.load() &&
      get_parameter("enable_quad_real_execution").as_bool();
    if ((real_goal_active_.load() && !quad_arm_allowed) ||
        gripper_goal_active_[side]) {
      RCLCPP_WARN(get_logger(),
        "Dex1 %zu goal rejected: same gripper or non-quad arm already executing", side);
      return rclcpp_action::GoalResponse::REJECT;
    }
    {
      std::lock_guard<std::mutex> cmd_lock(cmd_mtx_);
      if (gripper_fault_latched_) {
        RCLCPP_ERROR(get_logger(),
          "Dex1 fault latched: inspect robot and restart this node before retry");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    gripper_goal_active_[side] = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_cancel(
    const std::shared_ptr<GripperGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_gripper_accepted(
    const std::shared_ptr<GripperGoalHandle> gh, std::size_t side)
  {
    std::lock_guard<std::mutex> lk(gripper_threads_mtx_);
    // Reuse one worker slot per gripper; join the previous completed goal.
    if (gripper_threads_[side].joinable()) {
      gripper_threads_[side].join();
    }
    gripper_threads_[side] = std::thread([this, gh, side]() {
      execute_gripper(gh, side);
    });
  }

  void execute_gripper(const std::shared_ptr<GripperGoalHandle> gh,
                       std::size_t side)
  {
    const std::size_t id = GRIPPER_MOTOR_IDS[side];
    const double target = opening_to_motor(gh->get_goal()->command.position);
    auto finish = [&](int outcome, double q, bool stalled) {
      auto result = std::make_shared<GripperAction::Result>();
      result->position = motor_to_opening(q);
      result->effort = 0.0;  // No calibrated grasp force is available.
      result->stalled = stalled;
      result->reached_goal = (outcome == 0 && !dry_run_);
      if (!dry_run_) {
        std::lock_guard<std::mutex> lk(session_mtx_);
        gripper_goal_active_[side] = false;
      }
      if (!gh->is_active()) return;
      if (outcome == 0) gh->succeed(result);
      else if (outcome == 1) gh->canceled(result);
      else gh->abort(result);
    };

    if (dry_run_) {
      RCLCPP_WARN(get_logger(),
        "DEX1 DRY RUN side=%zu target_opening=%.5f m; no LowCmd sent",
        side, gh->get_goal()->command.position);
      finish(0, target, false);
      return;
    }

    double measured = 0.0;
    double feed_age = 0.0;
    std::array<double, N> current_q{};
    uint8_t mode = 0;
    double state_age = 0.0;

    if (!gripper_feedback(side, measured, feed_age) ||
        !latest_state(current_q, mode, state_age) ||
        state_age > lowstate_timeout_sec_ ||
        !valid_body_hold_state(current_q) ||
        !std::isfinite(current_q[id]) ||
        std::abs(measured - current_q[id]) > 0.05 ||
        measured < -0.03 || measured > GRIPPER_MOTOR_STROKE_RAD + 0.05) {
      RCLCPP_ERROR(get_logger(),
        "Dex1 %zu NOT started: stale/mismatched feedback, body state, "
        "or motor outside nominal 0..5.4 rad stroke", side);
      finish(2, measured, false);
      return;
    }

    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      if (!command_enabled_) {
        desired_q_ = current_q;  // Hold all 29 body joints at measured q.
      }
      // First goal for this gripper begins with its *measured* motor q.
      desired_q_[id] = measured;
      gripper_control_initialized_[side] = true;
      command_enabled_ = true;
    }

    // Dex1 shares the arm_sdk message, so the channel must be blended in even
    // for a gripper-only goal. Every body joint is already seeded at its
    // measured q, and the release guard hands the arms back on exit.
    struct WeightRelease {
      std::atomic<float> & target;
      ~WeightRelease() { target.store(0.0f); }
    } weight_release{arm_sdk_weight_target_};
    arm_sdk_weight_target_.store(1.0f);

    RCLCPP_WARN(get_logger(),
      "DEX1 REAL side=%zu: motor %zu, %.4f -> %.4f rad. "
      "A full 35-slot LowCmd (including BODY HOLD) is now active.",
      side, id, measured, target);

    // A full 5.4-rad stroke needs at least 10.8 s at 0.5 rad/s;
    // the previous fixed 4-s timeout would incorrectly abort such goals.
    // Allow additional tracking and settling time without increasing speed.
    const double command_speed = std::max(0.05, gripper_speed_limit_rad_s_);
    const double expected_motion_sec = std::abs(target - measured) / command_speed;
    const double goal_timeout_sec = std::max(
      gripper_goal_timeout_sec_, 2.0 * expected_motion_sec + 3.0);
    RCLCPP_INFO(get_logger(),
      "DEX1 side=%zu nominal goal duration %.2f s; timeout %.2f s",
      side, expected_motion_sec, goal_timeout_sec);

    auto t0 = std::chrono::steady_clock::now();
    auto prev = t0;
    auto in_tol_since = t0;
    bool in_tol = false;
    auto last_progress = t0;
    double last_progress_q = measured;
    const auto tick = std::chrono::milliseconds(20);

    while (rclcpp::ok() && !stop_writer_.load()) {
      const auto now = std::chrono::steady_clock::now();
      double q = 0.0, age = 0.0;
      uint8_t m = 0;
      double a = 0.0;
      std::array<double, N> body{};
      if (!gripper_feedback(side, q, age) ||
          !latest_state(body, m, a) || a > lowstate_timeout_sec_ ||
          !valid_body_hold_state(body) ||
          !std::isfinite(body[id]) || std::abs(body[id] - q) > 0.05 ||
          q < -0.03 || q > GRIPPER_MOTOR_STROKE_RAD + 0.05) {
        // Stop all low-level publishing on a feedback fault.
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          command_enabled_ = false;
          gripper_fault_latched_ = true;
        }
        RCLCPP_ERROR(get_logger(), "Dex1 feedback fault: LowCmd stopped");
        finish(2, q, false);
        return;
      }

      if (gh->is_canceling()) {
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          desired_q_[id] = q;  // Hold measured position on cancellation.
        }
        finish(1, q, false);
        return;
      }

      bool writer_stopped = false;
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        writer_stopped = !command_enabled_ || gripper_fault_latched_;
        if (!writer_stopped) {
          const double dt = std::clamp(
            std::chrono::duration<double>(now - prev).count(), 0.0, 0.1);
          const double max_step = gripper_speed_limit_rad_s_ * dt;
          const double before = desired_q_[id];
          desired_q_[id] = before + std::clamp(target - before,
            -max_step, max_step);
        }
      }
      if (writer_stopped) {
        finish(2, q, false);
        return;
      }
      prev = now;

      auto fb = std::make_shared<GripperAction::Feedback>();
      fb->position = motor_to_opening(q);
      fb->effort = 0.0;  // Unknown; not a force measurement.
      fb->stalled = false;
      fb->reached_goal = std::abs(q - target) <= gripper_goal_tolerance_rad_;
      if (gh->is_active()) gh->publish_feedback(fb);

      if (std::abs(q - target) <= gripper_goal_tolerance_rad_) {
        if (!in_tol) { in_tol = true; in_tol_since = now; }
        if (std::chrono::duration<double>(now - in_tol_since).count() >= 0.2) {
          finish(0, q, false);
          return;
        }
      } else {
        in_tol = false;
      }

      if (std::abs(q - last_progress_q) > 0.01) {
        last_progress_q = q;
        last_progress = now;
      }
      const double elapsed = std::chrono::duration<double>(now - t0).count();
      // A long, slow stroke must not be marked stalled just because the
      // desired target is still ramping. Check for actual tracking lag.
      double commanded = q;
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        commanded = desired_q_[id];
      }
      const bool significant_tracking_lag =
        std::abs(commanded - q) > std::max(0.04, gripper_goal_tolerance_rad_);
      if (elapsed > goal_timeout_sec ||
          (!in_tol && elapsed > 1.0 && significant_tracking_lag &&
           std::chrono::duration<double>(now - last_progress).count() > 1.5)) {
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          desired_q_[id] = q;  // Stop pushing on timeout/stall.
          gripper_fault_latched_ = true;
          if (both_arm_goal_active_.load() &&
              get_parameter("enable_quad_real_execution").as_bool()) {
            command_enabled_ = false;  // Quad fault stops all outgoing LowCmd.
          }
        }
        RCLCPP_ERROR(get_logger(), "Dex1 %zu stalled or goal timed out", side);
        finish(2, q, true);
        return;
      }
      std::this_thread::sleep_for(tick);
    }
    finish(2, measured, false);
  }

  static int64_t duration_ns(const builtin_interfaces::msg::Duration & d)
  {
    return static_cast<int64_t>(d.sec) * 1000000000LL +
           static_cast<int64_t>(d.nanosec);
  }

  static double duration_sec(const builtin_interfaces::msg::Duration & d)
  {
    return static_cast<double>(d.sec) +
           static_cast<double>(d.nanosec) * 1e-9;
  }

  static bool finite_vec(const std::vector<double> & v)
  {
    return std::all_of(v.begin(), v.end(),
      [](double x) { return std::isfinite(x); });
  }

  bool validate_message(
    const FollowJT::Goal & goal,
    const std::vector<std::string> & expected,
    std::string & err) const
  {
    const auto & tr = goal.trajectory;

    if (tr.points.empty()) {
      err = "trajectory has no points";
      return false;
    }

    if (tr.joint_names.size() != expected.size()) {
      err = "joint_names count does not match configured controller";
      return false;
    }

    std::unordered_set<std::string> recv;
    for (const auto & n : tr.joint_names) {
      if (!recv.insert(n).second) {
        err = "duplicate joint: " + n;
        return false;
      }
    }

    std::unordered_set<std::string> exp(expected.begin(), expected.end());
    for (const auto & n : tr.joint_names) {
      if (!exp.count(n)) {
        err = "unexpected joint: " + n;
        return false;
      }
    }
    for (const auto & n : expected) {
      if (!recv.count(n)) {
        err = "missing joint: " + n;
        return false;
      }
    }

    int64_t prev = -1;
    for (std::size_t i = 0; i < tr.points.size(); ++i) {
      const auto & p = tr.points[i];

      if (p.positions.size() != tr.joint_names.size()) {
        err = "invalid positions length at point " + std::to_string(i);
        return false;
      }

      if (!p.velocities.empty() &&
          p.velocities.size() != tr.joint_names.size()) {
        err = "invalid velocities length at point " + std::to_string(i);
        return false;
      }

      if (!finite_vec(p.positions) ||
          (!p.velocities.empty() && !finite_vec(p.velocities))) {
        err = "NaN/Inf at point " + std::to_string(i);
        return false;
      }

      const int64_t t = duration_ns(p.time_from_start);
      if (t < 0 || (i > 0 && t <= prev)) {
        err = "time_from_start is invalid/non-increasing";
        return false;
      }
      prev = t;
    }

    return true;
  }

  std::vector<std::size_t> index_map(
    const std::vector<std::string> & incoming,
    const std::vector<std::string> & canonical) const
  {
    std::unordered_map<std::string, std::size_t> m;
    for (std::size_t i = 0; i < incoming.size(); ++i) m[incoming[i]] = i;

    std::vector<std::size_t> out;
    out.reserve(canonical.size());
    for (const auto & n : canonical) out.push_back(m.at(n));
    return out;
  }

  std::vector<double> reorder(
    const std::vector<double> & in,
    const std::vector<std::size_t> & map) const
  {
    std::vector<double> out;
    out.reserve(map.size());
    for (const auto i : map) out.push_back(in[i]);
    return out;
  }

  // Joint IDs are fixed by G1-D hardware; check each joint's own official
  // bound. Works with left_arm (7), right_arm (7), and both_arms (14).
  std::pair<double, double> arm_bounds_for_id(int id) const
  {
    if (id >= 15 && id <= 21) {
      const auto j = static_cast<std::size_t>(id - 15);
      return {LEFT_ARM_MIN.at(j), LEFT_ARM_MAX.at(j)};
    }
    if (id >= 22 && id <= 28) {
      const auto j = static_cast<std::size_t>(id - 22);
      return {RIGHT_ARM_MIN.at(j), RIGHT_ARM_MAX.at(j)};
    }
    throw std::out_of_range("unexpected G1-D arm motor index");
  }

  bool validate_real_state(
    const FollowJT::Goal & goal,
    const std::vector<std::string> & joints,
    const std::vector<int> & ids,
    std::string & err)
  {
    std::array<double, N> q{};
    uint8_t mode = 0;
    double age = 0.0;

    if (!latest_state(q, mode, age)) {
      err = "no valid LowState";
      return false;
    }
    if (age > lowstate_timeout_sec_) {
      err = "LowState stale: " + std::to_string(age) + " s";
      return false;
    }

    const auto map = index_map(goal.trajectory.joint_names, joints);
    const auto q0 = reorder(goal.trajectory.points.front().positions, map);

    double max_start_err = 0.0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      max_start_err = std::max(
        max_start_err,
        std::abs(q0[i] - q.at(static_cast<std::size_t>(ids[i]))));
    }

    if (max_start_err > max_start_error_rad_) {
      err = "start-state mismatch: " + std::to_string(max_start_err) +
            " rad > " + std::to_string(max_start_error_rad_) + " rad";
      return false;
    }

    if (!valid_body_hold_state(q)) {
      err = "actual arm feedback outside official G1-D joint limits";
      return false;
    }

    // No total-motion 15-degree cap. Instead validate every arm joint against
    // the model-specific bounds in Unitree's official G1-D URDF.
    // Sampling each interpolation segment is necessary: MoveIt often supplies
    // endpoint velocities and cubic Hermite interpolation may overshoot points.
    const auto & pts = goal.trajectory.points;
    if (arm_max_trajectory_speed_rad_s_ <= 0.0 ||
        !std::isfinite(arm_max_trajectory_speed_rad_s_)) {
      err = "arm_max_trajectory_speed_rad_s must be finite and > 0";
      return false;
    }
    std::vector<double> previous = sample(goal, map, 0.0);
    double previous_t = 0.0;
    for (std::size_t seg = 0; seg < pts.size(); ++seg) {
      const double t1 = duration_sec(pts[seg].time_from_start);
      const double t0 = seg == 0 ? 0.0 :
        duration_sec(pts[seg - 1].time_from_start);
      const double dt = t1 - t0;
      if (dt < 0.0) {
        err = "trajectory timestamps invalid";
        return false;
      }
      const std::size_t n = std::max<std::size_t>(
        10, static_cast<std::size_t>(std::ceil(dt / 0.02)));
      if (n > 100000) {
        err = "trajectory segment excessively long";
        return false;
      }
      for (std::size_t step = 0; step <= n; ++step) {
        const double t = t0 + dt * static_cast<double>(step) /
          static_cast<double>(n);
        const auto positions = sample(goal, map, t);
        for (std::size_t j = 0; j < positions.size(); ++j) {
          const auto limits = arm_bounds_for_id(ids[j]);
          if (!std::isfinite(positions[j]) ||
              positions[j] < limits.first || positions[j] > limits.second) {
            err = "official G1-D URDF position limit violated at joint " +
              joints[j] + " (t=" + std::to_string(t) +
              ", q=" + std::to_string(positions[j]) +
              ", allowed=" + std::to_string(limits.first) + ".." +
              std::to_string(limits.second) + " rad)";
            return false;
          }
          if (t > previous_t + 1e-9 &&
              std::abs(positions[j] - previous[j]) / (t - previous_t) >
                arm_max_trajectory_speed_rad_s_ + 1e-6) {
            err = "arm command-speed guard exceeded at " + joints[j] +
              ": limit " + std::to_string(arm_max_trajectory_speed_rad_s_) +
              " rad/s (separate conservative setting, not URDF motor rating)";
            return false;
          }
        }
        if (t > previous_t + 1e-9) {
          previous = positions;
          previous_t = t;
        }
      }
    }

    return true;
  }

  std::vector<double> sample(
    const FollowJT::Goal & goal,
    const std::vector<std::size_t> & map,
    double t) const
  {
    const auto & pts = goal.trajectory.points;

    if (pts.size() == 1 || t <= duration_sec(pts.front().time_from_start))
      return reorder(pts.front().positions, map);

    const double tf = duration_sec(pts.back().time_from_start);
    if (t >= tf) return reorder(pts.back().positions, map);

    std::size_t hi = 1;
    while (hi < pts.size() &&
           duration_sec(pts[hi].time_from_start) < t) ++hi;

    const std::size_t lo = hi - 1;
    const double t0 = duration_sec(pts[lo].time_from_start);
    const double t1 = duration_sec(pts[hi].time_from_start);
    const double dt = t1 - t0;
    const double s = std::clamp((t - t0) / dt, 0.0, 1.0);

    const auto q0 = reorder(pts[lo].positions, map);
    const auto q1 = reorder(pts[hi].positions, map);
    std::vector<double> out(q0.size(), 0.0);

    const bool have_v =
      pts[lo].velocities.size() == goal.trajectory.joint_names.size() &&
      pts[hi].velocities.size() == goal.trajectory.joint_names.size();

    if (have_v) {
      const auto v0 = reorder(pts[lo].velocities, map);
      const auto v1 = reorder(pts[hi].velocities, map);

      const double s2 = s*s, s3 = s2*s;
      const double h00 = 2*s3 - 3*s2 + 1;
      const double h10 = s3 - 2*s2 + s;
      const double h01 = -2*s3 + 3*s2;
      const double h11 = s3 - s2;

      for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = h00*q0[i] + h10*dt*v0[i] + h01*q1[i] + h11*dt*v1[i];
    } else {
      for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = q0[i] + s*(q1[i] - q0[i]);
    }

    return out;
  }

  void print_positions(
    const std::string & label,
    const std::vector<std::string> & names,
    const std::vector<double> & q)
  {
    RCLCPP_INFO(get_logger(), "%s", label.c_str());
    for (std::size_t i = 0; i < names.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  %-32s %.6f rad",
                  names[i].c_str(), q[i]);
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJT::Goal> goal,
    const std::string & arm,
    const std::vector<std::string> & joints)
  {
    std::string err;
    if (!validate_message(*goal, joints, err)) {
      RCLCPP_ERROR(get_logger(), "[%s] reject: %s", arm.c_str(), err.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (dry_run_) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    if (!enable_real_execution_) {
      RCLCPP_ERROR(get_logger(), "enable_real_execution=false; rejecting real goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (arm == "both_arms" &&
        !get_parameter("enable_both_arms_real_execution").as_bool()) {
      RCLCPP_ERROR(get_logger(),
        "both_arms real execution NOT ENABLED; 14-joint plan only. "
        "Explicitly set enable_both_arms_real_execution=true after validation.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lk(session_mtx_);
    if (gripper_goal_active_[0] || gripper_goal_active_[1]) {
      RCLCPP_ERROR(get_logger(), "arm goal blocked: Dex1 goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!real_goal_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_ERROR(get_logger(), "another real goal is already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Under session_mtx_: the gripper admission test sees an atomic, valid
    // indication that a coordinated both_arms goal is active.
    both_arm_goal_active_.store(arm == "both_arms");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandle> gh,
    const std::string & arm,
    const std::vector<std::string> & joints,
    const std::vector<int> & ids)
  {
    std::thread([this, gh, arm, joints, ids]() {
      execute(gh, arm, joints, ids);
    }).detach();
  }

  void execute(
    const std::shared_ptr<GoalHandle> gh,
    const std::string arm,
    const std::vector<std::string> joints,
    const std::vector<int> ids)
  {
    const auto goal = gh->get_goal();
    const auto map = index_map(goal->trajectory.joint_names, joints);

    const auto first_q = reorder(goal->trajectory.points.front().positions, map);
    const auto last_q  = reorder(goal->trajectory.points.back().positions, map);

    RCLCPP_INFO(get_logger(), "Trajectory points: %zu", goal->trajectory.points.size());
    print_positions("First point:", joints, first_q);
    print_positions("Last point:", joints, last_q);

    if (dry_run_) {
      auto result = std::make_shared<FollowJT::Result>();
      result->error_code = FollowJT::Result::SUCCESSFUL;
      result->error_string = "dry-run successful";
      RCLCPP_INFO(get_logger(), "DRY RUN ONLY: no Unitree LowCmd sent.");
      gh->succeed(result);
      return;
    }

    std::string err;
    if (!validate_real_state(*goal, joints, ids, err)) {
      both_arm_goal_active_.store(false);
      real_goal_active_.store(false);

      auto result = std::make_shared<FollowJT::Result>();
      result->error_code = FollowJT::Result::INVALID_GOAL;
      result->error_string = err;

      RCLCPP_ERROR(get_logger(), "REAL goal aborted before LowCmd: %s", err.c_str());
      gh->abort(result);
      return;
    }

    std::array<double, N> current_q{};
    uint8_t mode = 0;
    double age = 0.0;

    if (!latest_state(current_q, mode, age)) {
      both_arm_goal_active_.store(false);
      real_goal_active_.store(false);

      auto result = std::make_shared<FollowJT::Result>();
      result->error_code = FollowJT::Result::INVALID_GOAL;
      result->error_string = "no LowState immediately before execution";
      gh->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      // Do not overwrite Dex1 target positions during quad operation.
      // Initialize the body only; each Dex1 thread owns indices 31/33.
      for (std::size_t i = 0; i < BODY_MOTOR_COUNT; ++i) {
        desired_q_[i] = current_q[i];
      }
      command_enabled_ = true;
    }

    // Hand the arms over exactly the way the official examples do: every joint
    // is already seeded with its measured q, then the arm_sdk weight ramps
    // 0 -> 1. Releasing the weight on every exit path restores the robot's own
    // arm controller, so an abort or a cancel never drops the arms.
    struct WeightRelease {
      std::atomic<float> & target;
      ~WeightRelease() { target.store(0.0f); }
    } weight_release{arm_sdk_weight_target_};

    arm_sdk_weight_target_.store(1.0f);
    {
      const auto ramp_start = std::chrono::steady_clock::now();
      while (rclcpp::ok() && arm_sdk_weight_.load() < 0.98f) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const double waited = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - ramp_start).count();
        if (waited > 5.0) {
          real_goal_active_.store(false);
          auto result = std::make_shared<FollowJT::Result>();
          result->error_code = FollowJT::Result::INVALID_GOAL;
          result->error_string =
            "arm_sdk weight did not reach 1.0; arms not handed over";
          RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
          gh->abort(result);
          return;
        }
      }
    }

    RCLCPP_WARN(get_logger(),
      "REAL EXECUTION STARTED on %s (arm_sdk weight %.2f). "
      "Keep the hardware E-stop available.",
      arm.c_str(), static_cast<double>(arm_sdk_weight_.load()));

    const double tf = duration_sec(goal->trajectory.points.back().time_from_start);
    const auto start = std::chrono::steady_clock::now();
    const auto period =
      std::chrono::duration<double, std::milli>(control_period_ms_);

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        std::array<double, N> q{};
        uint8_t m = 0;
        double a = 0.0;

        if (latest_state(q, m, a)) {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          // Cancel the arm trajectory without clobbering gripper targets.
          for (std::size_t i = 0; i < BODY_MOTOR_COUNT; ++i) {
            desired_q_[i] = q[i];
          }
          // Never automatically re-enable a writer latched off by a fault.
        }

        both_arm_goal_active_.store(false);
      real_goal_active_.store(false);

        auto result = std::make_shared<FollowJT::Result>();
        result->error_code = FollowJT::Result::SUCCESSFUL;
        result->error_string =
          "trajectory canceled; hold target changed to latest LowState";
        gh->canceled(result);
        return;
      }

      std::array<double, N> qcheck{};
      uint8_t mcheck = 0;
      double agecheck = 0.0;

      if (!latest_state(qcheck, mcheck, agecheck) ||
          agecheck > lowstate_timeout_sec_) {
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          command_enabled_ = false;
        }

        both_arm_goal_active_.store(false);
      real_goal_active_.store(false);

        auto result = std::make_shared<FollowJT::Result>();
        result->error_code = FollowJT::Result::INVALID_GOAL;
        result->error_string =
          "LowState missing/stale during execution; LowCmd stopped";
        RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
        gh->abort(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        if (!command_enabled_ || gripper_fault_latched_) {
          both_arm_goal_active_.store(false);
          real_goal_active_.store(false);
          auto result = std::make_shared<FollowJT::Result>();
          result->error_code = FollowJT::Result::INVALID_GOAL;
          result->error_string = "LowCmd writer stopped or Dex1 fault latched during arm execution";
          RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
          gh->abort(result);
          return;
        }
      }

      const double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();

      const auto arm_q = sample(*goal, map, t);
      bool command_valid = true;
      for (std::size_t j = 0; j < ids.size(); ++j) {
        const auto limits = arm_bounds_for_id(ids[j]);
        if (!std::isfinite(arm_q[j]) ||
            arm_q[j] < limits.first || arm_q[j] > limits.second) {
          command_valid = false;
          break;
        }
      }
      if (!command_valid) {
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          command_enabled_ = false;
        }
        both_arm_goal_active_.store(false);
      real_goal_active_.store(false);
        auto result = std::make_shared<FollowJT::Result>();
        result->error_code = FollowJT::Result::INVALID_GOAL;
        result->error_string = "runtime official G1-D joint limit violation; LowCmd stopped";
        RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
        gh->abort(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        for (std::size_t i = 0; i < ids.size(); ++i) {
          desired_q_.at(static_cast<std::size_t>(ids[i])) = arm_q[i];
        }
      }

      if (t >= tf) break;

      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }

    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      for (std::size_t i = 0; i < ids.size(); ++i) {
        desired_q_.at(static_cast<std::size_t>(ids[i])) = last_q[i];
      }
    }

    // Keep holding the final target, but do not report SUCCESS until the
    // measured LowState has actually converged to the commanded final point.
    RCLCPP_INFO(
      get_logger(),
      "Trajectory time finished. Waiting for real joint convergence...");

    const auto settle_start = std::chrono::steady_clock::now();
    auto within_tolerance_since = settle_start;
    bool within_tolerance = false;
    bool settled = false;
    double final_max_error = 0.0;

    while (rclcpp::ok()) {
      std::array<double, N> measured_q{};
      uint8_t measured_mode = 0;
      double measured_age = 0.0;

      if (!latest_state(measured_q, measured_mode, measured_age) ||
          measured_age > lowstate_timeout_sec_) {
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          command_enabled_ = false;
        }

        both_arm_goal_active_.store(false);
      real_goal_active_.store(false);

        auto result = std::make_shared<FollowJT::Result>();
        result->error_code = FollowJT::Result::INVALID_GOAL;
        result->error_string =
          "LowState missing/stale while waiting for final convergence; LowCmd stopped";
        RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
        gh->abort(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        if (!command_enabled_ || gripper_fault_latched_) {
          both_arm_goal_active_.store(false);
          real_goal_active_.store(false);
          auto result = std::make_shared<FollowJT::Result>();
          result->error_code = FollowJT::Result::INVALID_GOAL;
          result->error_string = "LowCmd writer stopped or Dex1 fault latched during settling";
          RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
          gh->abort(result);
          return;
        }
      }

      double max_error = 0.0;
      for (std::size_t i = 0; i < ids.size(); ++i) {
        const double e = std::abs(
          measured_q.at(static_cast<std::size_t>(ids[i])) - last_q[i]);
        max_error = std::max(max_error, e);
      }
      final_max_error = max_error;

      const auto now = std::chrono::steady_clock::now();

      if (max_error <= goal_tolerance_rad_) {
        if (!within_tolerance) {
          within_tolerance = true;
          within_tolerance_since = now;
        }

        const double stable_for =
          std::chrono::duration<double>(now - within_tolerance_since).count();

        if (stable_for >= settle_duration_sec_) {
          settled = true;
          break;
        }
      } else {
        within_tolerance = false;
      }

      const double waited =
        std::chrono::duration<double>(now - settle_start).count();

      if (waited >= settle_timeout_sec_) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    both_arm_goal_active_.store(false);
      real_goal_active_.store(false);

    if (!settled) {
      auto result = std::make_shared<FollowJT::Result>();
      result->error_code = FollowJT::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string =
        "final real joint state did not settle within tolerance before timeout";

      RCLCPP_ERROR(
        get_logger(),
        "REAL trajectory reached time end but did not settle: "
        "max_error=%.6f rad, tolerance=%.6f rad, timeout=%.3f s. "
        "Final-position hold remains active.",
        final_max_error,
        goal_tolerance_rad_,
        settle_timeout_sec_);

      // Diagnosis only: report each arm joint's final tracking error and
      // whether the LowCmd writer still has command output enabled.
      std::array<double, N> cmd_snapshot{};
      bool writer_enabled = false;
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        cmd_snapshot = desired_q_;
        writer_enabled = command_enabled_;
      }
      RCLCPP_ERROR(get_logger(),
        "FINAL_TRACKING_DIAGNOSTIC writer_enabled=%s (command-vs-LowState)",
        writer_enabled ? "true" : "false");
      std::array<double, N> actual_snapshot{};
      uint8_t state_mode = 0;
      double state_age = 0.0;
      if (latest_state(actual_snapshot, state_mode, state_age)) {
        for (std::size_t i = 0; i < ids.size(); ++i) {
          const auto id = static_cast<std::size_t>(ids[i]);
          RCLCPP_ERROR(get_logger(),
            "FINAL_TRACKING_DIAGNOSTIC %s goal=%.6f cmd=%.6f actual=%.6f "
            "error=%.6f rad state_age=%.3f s",
            joints[i].c_str(), last_q[i], cmd_snapshot[id],
            actual_snapshot[id], cmd_snapshot[id] - actual_snapshot[id],
            state_age);
        }
      }
      gh->abort(result);
      return;
    }

    auto result = std::make_shared<FollowJT::Result>();
    result->error_code = FollowJT::Result::SUCCESSFUL;
    result->error_string =
      "real trajectory completed and measured joints converged; "
      "final hold remains active while process runs";
    gh->succeed(result);

    RCLCPP_WARN(
      get_logger(),
      "REAL EXECUTION COMPLETE. Measured joints converged "
      "(max_error=%.6f rad). Final-position hold remains active.",
      final_max_error);
  }
};

int main(int argc, char ** argv)
{
  if (!lzh_ros_environment_is_valid()) {
    return 2;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1DTrajectoryActionServer>());
  rclcpp::shutdown();
  return 0;
}
