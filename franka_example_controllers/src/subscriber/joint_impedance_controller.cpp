#include <franka_example_controllers/subscriber/joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>

#include <Eigen/Eigen>
#include "sensor_msgs/msg/joint_state.hpp"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

controller_interface::return_type
JointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  Eigen::Map<const Vector7d> coriolis(
    franka_robot_model_->getCoriolisForceVector().data());
  const auto& robot_state = *franka_robot_model_->getRobotState();
  /*
  The implementation uses a first-order low-pass filter.
    y[n] = (1-alpha) * y[n-1] + alpha * x[n]
  where
    alpha = (2 * M_PI * fc * T) / (1 + 2 * M_PI * fc * T)
  fc is the cutoff frequency and T is the sampling time. 
  */ 
  dq_filtered_ = (1.0 - alpha_) * dq_filtered_ + alpha_ * dq_;
  
  // Saturate the tracking errors, then reconstruct desired position/velocity.
  Vector7d q_error = (q_d_target_ - q_)
    .cwiseMin(Vector7d::Constant(pos_saturation_))
    .cwiseMax(Vector7d::Constant(-pos_saturation_));
  Vector7d dq_error = (dq_d_target_ - dq_filtered_)
    .cwiseMin(Vector7d::Constant(vel_saturation_))
    .cwiseMax(Vector7d::Constant(-vel_saturation_));

  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_error) + d_gains_.cwiseProduct(dq_error) + coriolis;

  std::array<double, 7> tau_d_calculated_array{};
  for (int i = 0; i < num_joints; ++i) {
    tau_d_calculated_array[i] = tau_d_calculated(i);
  }
  const std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(tau_d_calculated_array, robot_state.tau_J_d);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_saturated[i]);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn
JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    sub_desired_joint_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_impedance/joints_desired", 1,
      std::bind(&JointImpedanceController::desiredJointCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  q_d_target_.setZero();
  q_d_.setZero();
  dq_d_target_.setZero();
  dq_d_.setZero();
  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  q_d_ = q_;
  q_d_target_ = q_;
  dq_d_.setZero();
  dq_d_target_.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "JointImpedanceController on_activate:");
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  q_      = " << q_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  k_gains = " << k_gains_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  d_gains = " << d_gains_.transpose());

  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointImpedanceController::desiredJointCallback(
  const sensor_msgs::msg::JointState& msg) {
  if (msg.position.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %d desired joint positions, but received %zu.",
                 num_joints, msg.position.size());
    return;
  }
  if (!msg.velocity.empty() && msg.velocity.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %d desired joint velocities, but received %zu.",
                 num_joints, msg.velocity.size());
    return;
  }

  for (auto i = 0; i < num_joints; ++i) {
    q_d_target_(i) = msg.position[i];
    dq_d_target_(i) = msg.velocity.empty() ? 0.0 : msg.velocity[i];
  }
}

std::array<double, 7> JointImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) const {
  std::array<double, 7> tau_d_saturated{};
  for (int i = 0; i < num_joints; ++i) {
    const double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] +
        std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)