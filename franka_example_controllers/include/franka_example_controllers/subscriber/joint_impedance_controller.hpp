#pragma once

#include <array>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

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

  const double alpha_ = 0.24; // low-pass filter coefficient. alpha = (2 * M_PI * fc * T) / (1 + 2 * M_PI * fc * T)
  // fc is the cutoff frequency (50 Hz) and T is the sampling time (1 ms). 
  const double pos_saturation_ = 0.2; // position error saturation in radians
  const double vel_saturation_ = 0.5; // velocity error saturation in radians/s
  const double delta_tau_max_ = 1.0;  // max torque-rate step per control cycle
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d q_d_;
  Vector7d q_d_target_;
  Vector7d dq_d_;
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
};

}  // namespace franka_example_controllers
