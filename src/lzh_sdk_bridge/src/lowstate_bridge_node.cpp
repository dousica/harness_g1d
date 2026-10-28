#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Unitree SDK2
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>


class G1DLowStateBridge : public rclcpp::Node
{
public:
    G1DLowStateBridge()
    : Node("lzh_lowstate_bridge")
    {
        // ROS2参数：连接G1-D所使用的网卡
        this->declare_parameter<std::string>(
            "network_interface",
            "eno2");

        network_interface_ =
            this->get_parameter("network_interface")
                .as_string();

        // ROS2 JointState发布器
        joint_state_publisher_ =
            this->create_publisher<
                sensor_msgs::msg::JointState>(
                "/joint_states",
                10);

        dex1_motor_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "/lzh_dex1/motor_states", 10);

        // 初始化 Unitree SDK2 DDS
        unitree::robot::ChannelFactory::Instance()
            ->Init(0, network_interface_);

        // 订阅 G1-D LowState
        lowstate_subscriber_.reset(
            new unitree::robot::ChannelSubscriber<
                unitree_hg::msg::dds_::LowState_>(
                    "rt/lowstate"));

        lowstate_subscriber_->InitChannel(
            std::bind(
                &G1DLowStateBridge::LowStateHandler,
                this,
                std::placeholders::_1),
            1);

        RCLCPP_INFO(
            this->get_logger(),
            "G1-D LowState bridge started on interface: %s",
            network_interface_.c_str());
    }


private:
    void LowStateHandler(const void * message)
    {
        const auto * low_state =
            static_cast<
                const unitree_hg::msg::dds_::LowState_ *>(
                    message);

        if (low_state == nullptr) {
            return;
        }

        sensor_msgs::msg::JointState joint_state;

        joint_state.header.stamp = this->now();


        // 目前先使用已经实机验证过的三个关节。
        // motor_xx 仍是临时ROS名称，
        // 等拿到正式G1-D URDF后换成官方joint name。


        // G1-D 双臂电机ID
static const std::array<int, 14> motor_ids = {
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28
};

// 对应官方 G1-D URDF joint names
static const std::array<std::string, 14> joint_names = {
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",

    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint"
};

joint_state.name.reserve(motor_ids.size());
joint_state.position.reserve(motor_ids.size());
joint_state.velocity.reserve(motor_ids.size());

for (size_t i = 0; i < motor_ids.size(); ++i)
{
    const int id = motor_ids[i];

    joint_state.name.push_back(
        joint_names[i]);

    joint_state.position.push_back(
        low_state->motor_state()[id].q());

    joint_state.velocity.push_back(
        low_state->motor_state()[id].dq());
}

// =====================================================
// Dex1-1 dual gripper visualization
// Preliminary mapping: motor feedback -> URDF finger joints
// Read-only. No motor commands.
// =====================================================

static const std::array<int, 2> gripper_ids = {31, 33};

static const std::array<std::array<std::string, 2>, 2>
    finger_names = {{
        {
            "left_dex1_1_Joint1_1",
            "left_dex1_1_Joint2_1"
        },
        {
            "right_dex1_1_Joint1_1",
            "right_dex1_1_Joint2_1"
        }
    }};

// Preliminary closed reference measured on the robot.
constexpr double kClosedMotorQ = 0.0001;

// Dex1-1 URDF prismatic joint limits (meters).
constexpr double kClosedFingerQ = 0.0245;
constexpr double kOpenFingerQ   = -0.0200;

// Preliminary motor stroke from Unitree Dex1-1 example.
constexpr double kMotorStroke = 5.4;

constexpr double kMetersPerRad =
    (kClosedFingerQ - kOpenFingerQ) / kMotorStroke;

for (size_t i = 0; i < gripper_ids.size(); ++i)
{
    const auto & motor =
        low_state->motor_state()[gripper_ids[i]];

    const bool valid =
        motor.motorstate() == 0 &&
        motor.vol() > 0.0f &&
        (motor.temperature()[0] != 0 ||
         motor.temperature()[1] != 0) &&
        std::isfinite(motor.q()) &&
        std::isfinite(motor.dq());

    if (!valid) {
        continue;
    }

    const double motor_delta =
        motor.q() - kClosedMotorQ;

    // Reject clearly unexpected readings during visualization.
    if (motor_delta < -0.05 ||
        motor_delta > kMotorStroke + 0.05)
    {
        continue;
    }

    // Positive motor displacement is provisionally
    // interpreted as gripper opening.
    const double finger_q = std::clamp(
        kClosedFingerQ - motor_delta * kMetersPerRad,
        kOpenFingerQ,
        kClosedFingerQ
    );

    const double finger_dq =
        -motor.dq() * kMetersPerRad;

    // Two symmetric sliding fingers.
    for (const auto & name : finger_names[i])
    {
        joint_state.name.push_back(name);
        joint_state.position.push_back(finger_q);
        joint_state.velocity.push_back(finger_dq);
    }
}
        joint_state_publisher_->publish(joint_state);

        // Raw Dex1 motor feedback (radians), NOT URDF finger positions.
        sensor_msgs::msg::JointState dex1_state;
        dex1_state.header.stamp = joint_state.header.stamp;

        static const std::array<int, 2> dex1_ids = {31, 33};
        static const std::array<std::string, 2> dex1_names = {
            "left_dex1_motor_raw",
            "right_dex1_motor_raw"
        };

        for (size_t i = 0; i < dex1_ids.size(); ++i) {
            const auto & motor =
                low_state->motor_state()[dex1_ids[i]];

            // Conservative preliminary feedback screening.
            const bool valid =
                motor.motorstate() == 0 &&
                motor.vol() > 0.0f &&
                (motor.temperature()[0] != 0 ||
                 motor.temperature()[1] != 0) &&
                std::isfinite(motor.q()) &&
                std::isfinite(motor.dq());

            if (!valid) {
                continue;
            }

            dex1_state.name.push_back(dex1_names[i]);
            dex1_state.position.push_back(motor.q());
            dex1_state.velocity.push_back(motor.dq());
            dex1_state.effort.push_back(motor.tau_est());
        }

        if (!dex1_state.name.empty()) {
            dex1_motor_publisher_->publish(dex1_state);
        }



        // 每500帧打印一次，避免终端刷屏
        frame_count_++;

        if (frame_count_ % 500 == 0) {

            RCLCPP_INFO(
                this->get_logger(),
                "q15=%.5f  q18=%.5f  q20=%.5f",
                low_state->motor_state()[15].q(),
                low_state->motor_state()[18].q(),
                low_state->motor_state()[20].q());
        }
    }


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        dex1_motor_publisher_;

    std::string network_interface_;

    size_t frame_count_{0};

    rclcpp::Publisher<
        sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;

    unitree::robot::ChannelSubscriberPtr<
        unitree_hg::msg::dds_::LowState_>
        lowstate_subscriber_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<G1DLowStateBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
