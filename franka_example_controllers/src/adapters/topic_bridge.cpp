#include "franka_example_controllers/adapters/topic_bridge.hpp"
#include <algorithm>

namespace franka_example_controllers {

TopicBridge::TopicBridge() : rclcpp::Node("topic_bridge") {
  // 订阅全局joint_states
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&TopicBridge::jointStatesCallback, this, std::placeholders::_1));

  // 重新发布到师兄控制器期望的话题名称（panda_1/panda_2）
  joint1_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/panda_1/panda_1_franka_state_controller/joint_states", 10);

  joint2_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/panda_2/panda_2_franka_state_controller/joint_states", 10);

  RCLCPP_INFO(this->get_logger(), "Topic Bridge started");
  RCLCPP_INFO(this->get_logger(), "Subscribing: /joint_states");
  RCLCPP_INFO(this->get_logger(), "Remapping: mj_left → panda_1, mj_right → panda_2");
}

void TopicBridge::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // 使用临时存储，确保关节按编号排序
  struct JointData {
    std::string name;
    double position;
    double velocity;
    double effort;
    int index;
  };

  std::vector<JointData> left_joints, right_joints;

  // 遍历所有关节，分离mj_left和mj_right
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];

    if (joint_name.find("mj_left_joint") == 0) {
      // 提取关节编号（1-7）
      std::string joint_num_str = joint_name.substr(13); // "mj_left_joint"长度为13
      int joint_num = std::stoi(joint_num_str);

      JointData jd;
      jd.name = "panda_1_joint" + joint_num_str;
      jd.position = msg->position[i];
      jd.velocity = (!msg->velocity.empty()) ? msg->velocity[i] : 0.0;
      jd.effort = (!msg->effort.empty()) ? msg->effort[i] : 0.0;
      jd.index = joint_num;
      left_joints.push_back(jd);
    }
    else if (joint_name.find("mj_right_joint") == 0) {
      // 提取关节编号（1-7）
      std::string joint_num_str = joint_name.substr(14); // "mj_right_joint"长度为14
      int joint_num = std::stoi(joint_num_str);

      JointData jd;
      jd.name = "panda_2_joint" + joint_num_str;
      jd.position = msg->position[i];
      jd.velocity = (!msg->velocity.empty()) ? msg->velocity[i] : 0.0;
      jd.effort = (!msg->effort.empty()) ? msg->effort[i] : 0.0;
      jd.index = joint_num;
      right_joints.push_back(jd);
    }
  }

  // 按关节编号排序
  std::sort(left_joints.begin(), left_joints.end(),
            [](const JointData& a, const JointData& b) { return a.index < b.index; });
  std::sort(right_joints.begin(), right_joints.end(),
            [](const JointData& a, const JointData& b) { return a.index < b.index; });

  // 创建排序后的消息
  sensor_msgs::msg::JointState msg1, msg2;
  msg1.header = msg->header;
  msg2.header = msg->header;

  for (const auto& jd : left_joints) {
    msg1.name.push_back(jd.name);
    msg1.position.push_back(jd.position);
    msg1.velocity.push_back(jd.velocity);
    msg1.effort.push_back(jd.effort);
  }

  for (const auto& jd : right_joints) {
    msg2.name.push_back(jd.name);
    msg2.position.push_back(jd.position);
    msg2.velocity.push_back(jd.velocity);
    msg2.effort.push_back(jd.effort);
  }

  // 发布重映射后的关节状态
  if (!msg1.name.empty()) {
    joint1_pub_->publish(msg1);
  }
  if (!msg2.name.empty()) {
    joint2_pub_->publish(msg2);
  }
}

}  // namespace franka_example_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<franka_example_controllers::TopicBridge>();

  RCLCPP_INFO(node->get_logger(),
              "Topic Bridge node started, spinning...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
