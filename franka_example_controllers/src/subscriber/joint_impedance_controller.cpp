#include <franka_example_controllers/subscriber/joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>
#include <vector>

#include <Eigen/Eigen>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

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
  const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  updateJointStates();
  const auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<const Vector7d> coriolis_map(coriolis_array.data());
  Vector7d coriolis = coriolis_map;
  const auto& robot_state = *franka_robot_model_->getRobotState();
  const double dt = period.seconds();

  // determine whether publishing is allowed this control cycle
  if (publish_rate_ <= 0.0) {
    publish_allowed_ = true;
  } else {
    publish_allowed_ = (publish_cycle_counter_ == 0);
  }

  /* State-Space Kinematic Filtering
     Estimated Delay:
     t_daly ~ d_filt/k_filt^2 * omega^2 (with feedforward velocity)
     t_delay ~ d_filt / k_filt (without feedforward velocity)
  */
  Vector7d ddq_filt = k_filt_.cwiseProduct(q_d_target_ - q_filt_) +
                      d_filt_.cwiseProduct(dq_d_target_ - dq_filt_);
  ddq_filt = ddq_filt.cwiseMin(ddq_max_).cwiseMax(-ddq_max_);

  dq_filt_ = dq_filt_ + ddq_filt * dt;
  dq_filt_ = dq_filt_.cwiseMin(dq_max_).cwiseMax(-dq_max_);

  q_filt_ = q_filt_ + dq_filt_ * dt;

  if (publish_allowed_) {
    publishJointState(time);
    publishFilteredState(ddq_filt, time);
    publishExternalWrench(robot_state, time);
  }

  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_filt_ - q_) + d_gains_.cwiseProduct(dq_filt_ - dq_) + coriolis;

  std::array<double, 7> tau_d_calculated_array{};
  for (int i = 0; i < num_joints; ++i) {
    tau_d_calculated_array[i] = tau_d_calculated(i);
  }
  const std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(tau_d_calculated_array, robot_state.tau_J_d);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_saturated[i]);
  }
  // advance cycle counter for publish scheduling
  if (publish_rate_ > 0.0 && publish_cycles_ > 0) {
    publish_cycle_counter_ = (publish_cycle_counter_ + 1) % publish_cycles_;
  }
  return controller_interface::return_type::OK;
}

CallbackReturn
JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("control_frequency", 1000.0);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("dq_max", std::vector<double>(num_joints, 0.5));
    auto_declare<std::vector<double>>("ddq_max", std::vector<double>(num_joints, 5.0));
      auto_declare<std::vector<double>>("k_filt", std::vector<double>(num_joints, 400.0));
      auto_declare<std::vector<double>>("d_filt", std::vector<double>(num_joints, 40.0));
        auto_declare<bool>("pub_filt_state", false);
        auto_declare<double>("publish_rate", 1000.0);
    // subscription to desired joints will be created in on_configure()
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
  control_frequency_ = get_node()->get_parameter("control_frequency").as_double();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
    // create subscription to desired joints under the arm namespace
    std::string desired_topic = "/" + arm_id_ + "/joints_desired";
    sub_desired_joint_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      desired_topic, 1,
      std::bind(&JointImpedanceController::desiredJointCallback, this, std::placeholders::_1)
    );
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto dq_max = get_node()->get_parameter("dq_max").as_double_array();
  auto ddq_max = get_node()->get_parameter("ddq_max").as_double_array();
    auto k_filt = get_node()->get_parameter("k_filt").as_double_array();
    auto d_filt = get_node()->get_parameter("d_filt").as_double_array();
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
  if (dq_max.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "dq_max should be of size %d but is of size %ld",
                 num_joints, dq_max.size());
    return CallbackReturn::FAILURE;
  }
  if (ddq_max.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "ddq_max should be of size %d but is of size %ld",
                 num_joints, ddq_max.size());
    return CallbackReturn::FAILURE;
  }
  if (k_filt.size() != static_cast<uint>(num_joints)) {
     RCLCPP_FATAL(get_node()->get_logger(), "k_filt should be of size %d but is of size %ld",
                 num_joints, k_filt.size());
    return CallbackReturn::FAILURE;
  }
  if (d_filt.size() != static_cast<uint>(num_joints)) {
     RCLCPP_FATAL(get_node()->get_logger(), "d_filt should be of size %d but is of size %ld",
                 num_joints, d_filt.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
    dq_max_(i) = dq_max.at(i);
    ddq_max_(i) = ddq_max.at(i);
    k_filt_(i) = k_filt.at(i);
    d_filt_(i) = d_filt.at(i);
    joint_names_.push_back(arm_id_ + "_joint" + std::to_string(i + 1));
  }
  q_d_target_.setZero();
  dq_d_target_.setZero();
  q_filt_.setZero();
  dq_filt_.setZero();
  publish_filt_state_ = get_node()->get_parameter("pub_filt_state").as_bool();
  // configure publish rate (Hz -> seconds)
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  if (publish_rate_ <= 0.0) {
    publish_cycles_configured_ = false;
    publish_cycle_counter_ = 0;
  } else {
    publish_cycles_ = std::max(1, static_cast<int>(std::round(control_frequency_ / publish_rate_)));
    const double adjusted_rate = control_frequency_ / static_cast<double>(publish_cycles_);
    if (std::fabs(adjusted_rate - publish_rate_) > 1e-6) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "publish_rate %.3f Hz is not an integer divisor of control_frequency %.3f Hz; using %d cycles => %.3f Hz.",
                  publish_rate_, control_frequency_, publish_cycles_, adjusted_rate);
    }
    publish_cycle_counter_ = 0;
    publish_cycles_configured_ = true;
  }
  if (publish_filt_state_) {
    std::string topic = "/" + arm_id_ + "/filtered_joint_states";
    auto filtered_pub = get_node()->create_publisher<sensor_msgs::msg::JointState>(topic, rclcpp::SystemDefaultsQoS());
    realtime_pub_filt_state_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(filtered_pub);
  }
  
  std::string joint_topic = "/" + arm_id_ + "/joint_states";
  auto joint_pub = get_node()->create_publisher<sensor_msgs::msg::JointState>(joint_topic, rclcpp::SystemDefaultsQoS());
  realtime_joint_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_pub);

  auto wrench_pub = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/" + arm_id_ + "/external_wrench", rclcpp::SystemDefaultsQoS());
  realtime_external_wrench_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(wrench_pub);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  q_filt_ = q_;
  q_d_target_ = q_;
  dq_filt_ = dq_;
  dq_d_target_.setZero();

  if (realtime_pub_filt_state_) {
    auto& msg = realtime_pub_filt_state_->msg_;
    msg.header.frame_id = arm_id_ + "_link0";
    msg.name = joint_names_;
    msg.position.resize(num_joints);
    msg.velocity.resize(num_joints);
    msg.effort.resize(num_joints);
  }
  if (realtime_external_wrench_publisher_) {
    realtime_external_wrench_publisher_->msg_.header.frame_id = arm_id_ + "_link0";
  }
  if (realtime_joint_state_publisher_) {
    auto& msg = realtime_joint_state_publisher_->msg_;
    msg.header.frame_id = arm_id_ + "_link0";
    msg.name = joint_names_;
    msg.position.resize(num_joints);
    msg.velocity.resize(num_joints);
    msg.effort.resize(num_joints);
  }

  RCLCPP_INFO(get_node()->get_logger(), "JointImpedanceController on_activate:");

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  k_gains = " << k_gains_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  d_gains = " << d_gains_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  k_filt  = " << k_filt_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  d_filt  = " << d_filt_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  dq_max  = " << dq_max_.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  ddq_max = " << ddq_max_.transpose());

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Current Joint Positions: " << q_.transpose());

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

void JointImpedanceController::publishFilteredState(const Vector7d& ddq_filt, const rclcpp::Time& stamp) {
  if (!publish_filt_state_ || !realtime_pub_filt_state_ || !realtime_pub_filt_state_->trylock()) {
    return;
  }

  auto& msg = realtime_pub_filt_state_->msg_;
  msg.header.stamp = stamp;
  for (int i = 0; i < num_joints; ++i) {
    msg.position[i] = q_filt_(i);
    msg.velocity[i] = dq_filt_(i);
    msg.effort[i] = ddq_filt(i);
  }
  realtime_pub_filt_state_->unlockAndPublish();
  // publishing happened on this cycle; nothing else to do (counter reset handled in update())
}

void JointImpedanceController::publishExternalWrench(const franka::RobotState& robot_state,
                                                     const rclcpp::Time& stamp) {
  if (!realtime_external_wrench_publisher_ || !realtime_external_wrench_publisher_->trylock()) {
    return;
  }

  auto& msg = realtime_external_wrench_publisher_->msg_;
  msg.header.stamp = stamp;
  msg.wrench.force.x = robot_state.O_F_ext_hat_K[0];
  msg.wrench.force.y = robot_state.O_F_ext_hat_K[1];
  msg.wrench.force.z = robot_state.O_F_ext_hat_K[2];
  msg.wrench.torque.x = robot_state.O_F_ext_hat_K[3];
  msg.wrench.torque.y = robot_state.O_F_ext_hat_K[4];
  msg.wrench.torque.z = robot_state.O_F_ext_hat_K[5];
  realtime_external_wrench_publisher_->unlockAndPublish();
  // publishing happened on this cycle; nothing else to do (counter advanced in update())
}

void JointImpedanceController::publishJointState(const rclcpp::Time& stamp) {
  if (!realtime_joint_state_publisher_ || !realtime_joint_state_publisher_->trylock()) {
    return;
  }

  auto& msg = realtime_joint_state_publisher_->msg_;
  msg.header.stamp = stamp;
  for (int i = 0; i < num_joints; ++i) {
    msg.position[i] = q_(i);
    msg.velocity[i] = dq_(i);
    msg.effort[i] = 0.0;
  }
  realtime_joint_state_publisher_->unlockAndPublish();
}

void JointImpedanceController::desiredJointCallback(
  const sensor_msgs::msg::JointState& msg) {
  if (msg.name.size() != msg.position.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected JointState names and positions to have the same size, but received %zu and %zu.",
                 msg.name.size(), msg.position.size());
    return;
  }
  if (!msg.velocity.empty() && msg.velocity.size() != msg.name.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected JointState velocity to be empty or the same size as name, but received %zu and %zu.",
                 msg.velocity.size(), msg.name.size());
    return;
  }

  for (int joint_index = 0; joint_index < num_joints; ++joint_index) {
    const std::string joint_name = arm_id_ + "_joint" + std::to_string(joint_index + 1);
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (it == msg.name.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing desired joint name '%s' in JointState message.",
                   joint_name.c_str());
      return;
    }

    const size_t msg_index = static_cast<size_t>(std::distance(msg.name.begin(), it));
    q_d_target_(joint_index) = msg.position.at(msg_index);
    dq_d_target_(joint_index) = msg.velocity.empty() ? 0.0 : msg.velocity.at(msg_index);
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