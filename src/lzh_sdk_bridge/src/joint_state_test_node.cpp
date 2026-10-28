#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateTestNode : public rclcpp::Node
{
public:
    JointStateTestNode()
    : Node("lzh_joint_state_test")
    {
        publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states",
                10);

        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &JointStateTestNode::publish_joint_state,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "G1-D JointState test node started.");
    }

private:
    void publish_joint_state()
    {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = this->now();

        // 目前只是ROS2接口测试，名称暂时使用电机ID
        // 后面拿到正式URDF后再换成官方joint name
        msg.name = {
            "motor_15",
            "motor_18",
            "motor_20"
        };

        // 使用实验1最开始记录的参考姿态附近数值
        msg.position = {
            -0.0305,
             1.42235,
             0.00799
        };

        msg.velocity = {
            0.0,
            0.0,
            0.0
        };

        publisher_->publish(msg);
    }

    rclcpp::Publisher<
        sensor_msgs::msg::JointState>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<JointStateTestNode>());

    rclcpp::shutdown();

    return 0;
}
