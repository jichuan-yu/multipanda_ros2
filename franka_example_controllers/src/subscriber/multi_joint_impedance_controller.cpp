#include <franka_example_controllers/subscriber/multi_joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>

#include <Eigen/Eigen>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MultiJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/effort");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
MultiJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
    for (const auto& franka_robot_model_name : arm_container_pair.second.franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(franka_robot_model_name);
    }
  }
  return config;
}

controller_interface::return_type MultiJointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();
  const double dt = period.seconds();
  const rclcpp::Time stamp = get_node()->now();

  if (publish_rate_ <= 0.0) {
    publish_allowed_ = true;
  } else {
    publish_allowed_ = (publish_cycle_counter_ == 0);
  }
  
  size_t k = 0;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    const auto coriolis_array = arm.franka_robot_model_->getCoriolisForceVector();
    Eigen::Map<const Vector7d> coriolis_map(coriolis_array.data());
    Vector7d coriolis = coriolis_map;

    Vector7d ddq_filt = arm.k_filt_.cwiseProduct(arm.q_d_target_ - arm.q_filt_) +
                        arm.d_filt_.cwiseProduct(arm.dq_d_target_ - arm.dq_filt_);
    ddq_filt = ddq_filt.cwiseMin(arm.ddq_max_).cwiseMax(-arm.ddq_max_);

    arm.dq_filt_ = arm.dq_filt_ + ddq_filt * dt;
    arm.dq_filt_ = arm.dq_filt_.cwiseMin(arm.dq_max_).cwiseMax(-arm.dq_max_);
    arm.q_filt_ = arm.q_filt_ + arm.dq_filt_ * dt;

    Vector7d tau_d_calculated =
        arm.k_gains_.cwiseProduct(arm.q_filt_ - arm.q_) +
        arm.d_gains_.cwiseProduct(arm.dq_filt_ - arm.dq_) + coriolis;

    if (publish_allowed_) {
      publishFilteredState(arm, ddq_filt, stamp);
      publishExternalWrench(arm, stamp);
      publishJointState(arm, stamp);
    }
    
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[k].set_value(tau_d_calculated(i));
      k++;
    }
  }
  if (publish_rate_ > 0.0 && publish_cycles_ > 0) {
    publish_cycle_counter_ = (publish_cycle_counter_ + 1) % publish_cycles_;
  }
  return controller_interface::return_type::OK;
}

CallbackReturn MultiJointImpedanceController::on_init() {
  try {
    rclcpp::Parameter arm_count;
    bool bHas_arm_count = get_node()->get_parameter("arm_count", arm_count);
    if(!bHas_arm_count){
      auto_declare<int>("arm_count", 0);
    }
    auto_declare<double>("publish_rate", 100.0);
    auto_declare<double>("control_frequency", 1000.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  try{
    num_robots = get_node()->get_parameter("arm_count").as_int();
  } catch(const rclcpp::ParameterTypeException& e){
    RCLCPP_FATAL(get_node()->get_logger(), "arm_count missing");
    return CallbackReturn::FAILURE;
  }

  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  control_frequency_ = get_node()->get_parameter("control_frequency").as_double();
  if (publish_rate_ > 0.0 && control_frequency_ > 0.0) {
    publish_cycles_ = std::max(1, static_cast<int>(std::round(control_frequency_ / publish_rate_)));
    publish_cycle_counter_ = 0;
    const double adjusted_rate = control_frequency_ / static_cast<double>(publish_cycles_);
    if (std::fabs(adjusted_rate - publish_rate_) > 1e-6) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "publish_rate %.3f Hz is not an integer divisor of control_frequency %.3f Hz; using %d cycles => %.3f Hz.",
                  publish_rate_, control_frequency_, publish_cycles_, adjusted_rate);
    }
  } else {
    publish_cycle_counter_ = 0;
  }

  for(int i = 1; i <= num_robots; i++){
    std::string arm_id_param_name = "arm_" + std::to_string(i) + ".arm_id";
    if(!get_node()->has_parameter(arm_id_param_name)){
        get_node()->declare_parameter<std::string>(arm_id_param_name, "");
    }
    std::string arm_id = get_node()->get_parameter(arm_id_param_name).as_string();
    arms_.insert(std::make_pair(arm_id, ArmContainer()));
  }
  
  int i = 1;
  for(auto& arm_container_pair : arms_){
    std::string prefix = "arm_" + std::to_string(i) + ".";
    auto &arm = arm_container_pair.second;
    arm.arm_id_ = arm_container_pair.first;
    
    arm.franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm.arm_id_ + "/robot_model", arm.arm_id_));

    if(!get_node()->has_parameter(prefix + "k_gains")){
        get_node()->declare_parameter<std::vector<double>>(prefix + "k_gains", {});
    }
    if(!get_node()->has_parameter(prefix + "d_gains")){
        get_node()->declare_parameter<std::vector<double>>(prefix + "d_gains", {});
    }
    if(!get_node()->has_parameter(prefix + "dq_max")){
      get_node()->declare_parameter<std::vector<double>>(prefix + "dq_max", std::vector<double>(num_joints, 0.5));
    }
    if(!get_node()->has_parameter(prefix + "ddq_max")){
      get_node()->declare_parameter<std::vector<double>>(prefix + "ddq_max", std::vector<double>(num_joints, 5.0));
    }
    if(!get_node()->has_parameter(prefix + "k_filt")){
      get_node()->declare_parameter<std::vector<double>>(prefix + "k_filt", std::vector<double>(num_joints, 400.0));
    }
    if(!get_node()->has_parameter(prefix + "d_filt")){
      get_node()->declare_parameter<std::vector<double>>(prefix + "d_filt", std::vector<double>(num_joints, 40.0));
    }
    if(!get_node()->has_parameter(prefix + "pub_filt_state")){
      get_node()->declare_parameter<bool>(prefix + "pub_filt_state", false);
    }

    auto k_gains = get_node()->get_parameter(prefix + "k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter(prefix + "d_gains").as_double_array();
    auto dq_max = get_node()->get_parameter(prefix + "dq_max").as_double_array();
    auto ddq_max = get_node()->get_parameter(prefix + "ddq_max").as_double_array();
    auto k_filt = get_node()->get_parameter(prefix + "k_filt").as_double_array();
    auto d_filt = get_node()->get_parameter(prefix + "d_filt").as_double_array();
    
    if (k_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (k_gains.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld", num_joints, k_gains.size());
      return CallbackReturn::FAILURE;
    }
    if (d_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (d_gains.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld", num_joints, d_gains.size());
      return CallbackReturn::FAILURE;
    }
    if (dq_max.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "dq_max should be of size %d but is of size %ld", num_joints, dq_max.size());
      return CallbackReturn::FAILURE;
    }
    if (ddq_max.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "ddq_max should be of size %d but is of size %ld", num_joints, ddq_max.size());
      return CallbackReturn::FAILURE;
    }
    if (k_filt.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_filt should be of size %d but is of size %ld", num_joints, k_filt.size());
      return CallbackReturn::FAILURE;
    }
    if (d_filt.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_filt should be of size %d but is of size %ld", num_joints, d_filt.size());
      return CallbackReturn::FAILURE;
    }
    for (int j = 0; j < num_joints; ++j) {
      arm.d_gains_(j) = d_gains.at(j);
      arm.k_gains_(j) = k_gains.at(j);
      arm.dq_max_(j) = dq_max.at(j);
      arm.ddq_max_(j) = ddq_max.at(j);
      arm.k_filt_(j) = k_filt.at(j);
      arm.d_filt_(j) = d_filt.at(j);
    }
    arm.q_.setZero();
    arm.dq_.setZero();
    arm.q_filt_.setZero();
    arm.dq_filt_.setZero();
    arm.q_d_target_.setZero();
    arm.dq_d_target_.setZero();
    arm.publish_filt_state_ = get_node()->get_parameter(prefix + "pub_filt_state").as_bool();
    if (arm.publish_filt_state_) {
      std::string topic = "/" + arm.arm_id_ + "/filtered_joint_states";
      auto filtered_pub = get_node()->create_publisher<sensor_msgs::msg::JointState>(topic, rclcpp::SystemDefaultsQoS());
      arm.realtime_filt_state_publisher_ =
          std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(filtered_pub);
    }
    std::string wrench_topic = "/" + arm.arm_id_ + "/external_wrench";
    auto wrench_pub = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, rclcpp::SystemDefaultsQoS());
    arm.realtime_external_wrench_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(wrench_pub);
    initializeJointStatePublisher(arm);
    std::string desired_topic = "/" + arm.arm_id_ + "/joints_desired";
    auto* arm_ptr = &arm;
    sub_desired_joint_[arm.arm_id_] = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        desired_topic, 1,
        [this, arm_ptr](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          this->desiredJointCallback(*msg, *arm_ptr);
        });
    i++;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.initial_q_ = arm.q_;
    arm.q_filt_ = arm.q_;
    arm.q_d_target_ = arm.q_;
    arm.dq_filt_ = arm.dq_;
    arm.dq_d_target_.setZero();
    arm.franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    if (arm.realtime_joint_state_publisher_) {
      auto &msg = arm.realtime_joint_state_publisher_->msg_;
      msg.header.frame_id = arm.arm_id_ + "_link0";
      msg.name = arm.joint_names_;
      msg.position.resize(num_joints);
      msg.velocity.resize(num_joints);
      msg.effort.resize(num_joints);
    }
    if (arm.realtime_filt_state_publisher_) {
      auto &msg = arm.realtime_filt_state_publisher_->msg_;
      msg.header.frame_id = arm.arm_id_ + "_link0";
      msg.name = arm.joint_names_;
      msg.position.resize(num_joints);
      msg.velocity.resize(num_joints);
      msg.effort.resize(num_joints);
    }
    if (arm.realtime_external_wrench_publisher_) {
      arm.realtime_external_wrench_publisher_->msg_.header.frame_id = arm.arm_id_ + "_link0";
    }
  }
  start_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
}

void MultiJointImpedanceController::initializeJointStatePublisher(ArmContainer& arm) {
  std::string joint_topic = "/" + arm.arm_id_ + "/joint_states";
  arm.joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      joint_topic, rclcpp::SystemDefaultsQoS());
  arm.realtime_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
          arm.joint_state_publisher_);
  arm.joint_names_.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    arm.joint_names_[i] = arm.arm_id_ + "_joint" + std::to_string(i + 1);
  }

  auto& msg = arm.realtime_joint_state_publisher_->msg_;
  msg.header.frame_id = arm.arm_id_ + "_link0";
  msg.name = arm.joint_names_;
  msg.position.resize(num_joints);
  msg.velocity.resize(num_joints);
  msg.effort.resize(num_joints);
}

void MultiJointImpedanceController::publishJointState(ArmContainer& arm, const rclcpp::Time& stamp) {
  if (!arm.realtime_joint_state_publisher_ || !arm.realtime_joint_state_publisher_->trylock()) {
    return;
  }

  auto& msg = arm.realtime_joint_state_publisher_->msg_;
  msg.header.stamp = stamp;
  for (int i = 0; i < num_joints; ++i) {
    msg.position[i] = arm.q_(i);
    msg.velocity[i] = arm.dq_(i);
    msg.effort[i] = 0.0;
  }
  arm.realtime_joint_state_publisher_->unlockAndPublish();
}

void MultiJointImpedanceController::publishFilteredState(ArmContainer& arm, const Vector7d& ddq_filt, const rclcpp::Time& stamp) {
  if (!arm.publish_filt_state_ || !arm.realtime_filt_state_publisher_ || !arm.realtime_filt_state_publisher_->trylock()) {
    return;
  }

  auto& msg = arm.realtime_filt_state_publisher_->msg_;
  msg.header.stamp = stamp;
  for (int i = 0; i < num_joints; ++i) {
    msg.position[i] = arm.q_filt_(i);
    msg.velocity[i] = arm.dq_filt_(i);
    msg.effort[i] = ddq_filt(i);
  }
  arm.realtime_filt_state_publisher_->unlockAndPublish();
}

void MultiJointImpedanceController::publishExternalWrench(ArmContainer& arm, const rclcpp::Time& stamp) {
  if (!arm.realtime_external_wrench_publisher_ || !arm.realtime_external_wrench_publisher_->trylock()) {
    return;
  }

  const auto& robot_state = *arm.franka_robot_model_->getRobotState();
  auto& msg = arm.realtime_external_wrench_publisher_->msg_;
  msg.header.stamp = stamp;
  msg.wrench.force.x = robot_state.O_F_ext_hat_K[0];
  msg.wrench.force.y = robot_state.O_F_ext_hat_K[1];
  msg.wrench.force.z = robot_state.O_F_ext_hat_K[2];
  msg.wrench.torque.x = robot_state.O_F_ext_hat_K[3];
  msg.wrench.torque.y = robot_state.O_F_ext_hat_K[4];
  msg.wrench.torque.z = robot_state.O_F_ext_hat_K[5];
  arm.realtime_external_wrench_publisher_->unlockAndPublish();
}

void MultiJointImpedanceController::updateJointStates() {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    size_t k = 0;
    for (size_t i = 0; i < state_interfaces_.size(); i++) {
      const auto& position_interface = state_interfaces_.at(i);
      if(position_interface.get_interface_name() == "position" && position_interface.get_prefix_name().find(arm_container_pair.first) != std::string::npos){
         arm.q_(k) = position_interface.get_value();
         k++;
         if(k==7) break;
      }
    }
    k = 0;
    for (size_t i = 0; i < state_interfaces_.size(); i++) {
      const auto& velocity_interface = state_interfaces_.at(i);
      if(velocity_interface.get_interface_name() == "velocity" && velocity_interface.get_prefix_name().find(arm_container_pair.first) != std::string::npos){
         arm.dq_(k) = velocity_interface.get_value();
         k++;
         if(k==7) break;
      }
    }
  }
}

void MultiJointImpedanceController::desiredJointCallback(const sensor_msgs::msg::JointState& msg,
                                                         ArmContainer& arm) {
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
    const std::string joint_name = arm.arm_id_ + "_joint" + std::to_string(joint_index + 1);
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (it == msg.name.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing desired joint name '%s' in JointState message.",
                   joint_name.c_str());
      return;
    }

    const size_t msg_index = static_cast<size_t>(std::distance(msg.name.begin(), it));
    arm.q_d_target_(joint_index) = msg.position.at(msg_index);
    arm.dq_d_target_(joint_index) = msg.velocity.empty() ? 0.0 : msg.velocity.at(msg_index);
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MultiJointImpedanceController,
                       controller_interface::ControllerInterface)
