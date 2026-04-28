#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace franka_example_controllers {

/**
 * @brief Topic Bridge Node
 *
 * 解决命名空间不匹配问题：
 * - 实际系统使用: mj_left/mj_right
 * - 师兄控制器期望: panda_1/panda_2
 *
 * 这个节点桥接关节状态话题，让师兄控制器能收到数据
 */
class TopicBridge : public rclcpp::Node {
 public:
  TopicBridge();
  ~TopicBridge() = default;

 private:
  // 订阅全局joint_states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  // 重新发布到师兄期望的话题名称
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint2_pub_;

  // 回调函数
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace franka_example_controllers
