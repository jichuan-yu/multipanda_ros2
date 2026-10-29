#pragma once

#include <string>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <realtime_tools/realtime_publisher.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace franka_example_controllers {

class MultiJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  struct ArmContainer{
    std::string arm_id_;
    Vector7d q_;
    Vector7d initial_q_;
    Vector7d dq_;
    Vector7d q_d_target_;
    Vector7d q_d_target_raw_;
    Vector7d dq_d_target_;
    Vector7d dq_d_target_raw_;
    Vector7d e_q_max_;
    Vector7d k_gains_;
    Vector7d d_gains_;
    double alpha_filt_ = 0.1;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_filt_state_publisher_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> realtime_external_wrench_publisher_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_joint_state_publisher_;
    std::vector<std::string> joint_names_;
  };

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  int num_robots;
  const int num_joints = 7;
  std::map<std::string, ArmContainer> arms_;
  rclcpp::Time start_time_;
    double publish_rate_ = 100.0;
    double control_frequency_ = 1000.0;
    int publish_cycles_ = 1;
    int publish_cycle_counter_ = 0;
    bool publish_allowed_ = true;
  void updateJointStates();
  void initializeJointStatePublisher(ArmContainer& arm);
  void publishJointState(ArmContainer& arm, const rclcpp::Time& stamp);
  void publishFilteredTarget(ArmContainer& arm, const rclcpp::Time& stamp);
  void publishExternalWrench(ArmContainer& arm, const rclcpp::Time& stamp);

  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
      sub_desired_joint_;
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg, ArmContainer& arm);
};

}  // namespace franka_example_controllers
