#include <franka_example_controllers/subscriber/multi_joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

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
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  
  size_t k = 0;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    Eigen::Map<const Vector7d> coriolis(
      arm.franka_robot_model_->getCoriolisForceVector().data());

    Vector7d q_goal = arm.q_des_;

    const double kAlpha = 0.99;
    arm.dq_filtered_ = (1 - kAlpha) * arm.dq_filtered_ + kAlpha * arm.dq_;
    Vector7d tau_d_calculated =
        arm.k_gains_.cwiseProduct(q_goal - arm.q_) + arm.d_gains_.cwiseProduct(-arm.dq_filtered_) + coriolis;
    
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[k].set_value(tau_d_calculated(i));
      k++;
    }
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
    
    sub_desired_joint_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dual_joint_impedance/joints_desired", 1,
      std::bind(&MultiJointImpedanceController::desiredJointCallback, this, std::placeholders::_1)
    );
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

    auto k_gains = get_node()->get_parameter(prefix + "k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter(prefix + "d_gains").as_double_array();
    
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
    for (int j = 0; j < num_joints; ++j) {
      arm.d_gains_(j) = d_gains.at(j);
      arm.k_gains_(j) = k_gains.at(j);
    }
    arm.dq_filtered_.setZero();
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
    arm.q_des_ = arm.q_;
    arm.franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  }
  start_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
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

void MultiJointImpedanceController::desiredJointCallback(const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data.size() >= num_robots * num_joints){
      int offset = 0;
      for(auto& arm_container_pair : arms_){
        auto &arm = arm_container_pair.second;
        for (auto i = 0; i < num_joints; ++i) {
          arm.q_des_(i) = msg.data[offset + i];
        }
        offset += num_joints;
      }
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MultiJointImpedanceController,
                       controller_interface::ControllerInterface)
