// lzh_test_pkg 的第一个节点：定时打印一句话，验证 ROS 2 工作空间跑通
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_environment_guard.hpp"

class LzhHelloNode : public rclcpp::Node
{
public:
  LzhHelloNode() : Node("lzh_hello_node")
  {
    // 定时器：每 2 秒回调一次
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Hello from %s!", this->get_name());
        count_++;
        RCLCPP_INFO(this->get_logger(), "计数 = %ld", count_);
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  long count_ = 0;
};

int main(int argc, char ** argv)
{
  if (!lzh_test_ros_environment_is_valid()) {
    return 2;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LzhHelloNode>());
  rclcpp::shutdown();
  return 0;
}
