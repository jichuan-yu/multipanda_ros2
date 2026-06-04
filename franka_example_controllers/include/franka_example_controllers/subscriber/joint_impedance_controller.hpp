#pragma once

#include <array>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <realtime_tools/realtime_publisher.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint impedance controller receives desired joint position.
 */
class JointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;

  Vector7d dq_max_;
  Vector7d ddq_max_;
  Vector7d k_filt_;
  Vector7d d_filt_; 
  const double delta_tau_max_ = 1.0;  // max torque-rate step per control cycle (1000 Nm/s)
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d q_filt_;
  Vector7d dq_filt_;
  Vector7d q_d_target_;
  Vector7d dq_d_target_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  rclcpp::Time start_time_;
  void updateJointStates();
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d) const;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_joint_;
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg);

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_pub_filt_state_;
  bool publish_filt_state_ = true;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> realtime_external_wrench_publisher_;
  std::vector<std::string> joint_names_;

  void publishFilteredState(const Vector7d& ddq_filt, const rclcpp::Time& stamp);
  void publishExternalWrench(const franka::RobotState& robot_state, const rclcpp::Time& stamp);
};

}  // namespace franka_example_controllers
