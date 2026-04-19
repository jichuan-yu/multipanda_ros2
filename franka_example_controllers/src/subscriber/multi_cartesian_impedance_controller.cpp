#include <franka_example_controllers/subscriber/multi_cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <franka/model.h>

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MultiCartesianImpedanceController::command_interface_configuration() const {
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
MultiCartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (const auto& franka_robot_model_name : arm_container_pair.second.franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(franka_robot_model_name);
    }
  }
  return config;
}

controller_interface::return_type MultiCartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  size_t k = 0;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    Eigen::Map<const Matrix4d> current(arm.franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
    Eigen::Vector3d current_position(current.block<3,1>(0,3));
    Eigen::Quaterniond current_orientation(current.block<3,3>(0,0));
    Eigen::Map<const Matrix7d> inertia(arm.franka_robot_model_->getMassMatrix().data());
    Eigen::Map<const Vector7d> coriolis(arm.franka_robot_model_->getCoriolisForceVector().data());
    Eigen::Matrix<double, 6, 7> jacobian(
        arm.franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
    Eigen::Map<const Vector7d> qD(arm.franka_robot_model_->getRobotState()->dq.data());
    Eigen::Map<const Vector7d> q(arm.franka_robot_model_->getRobotState()->q.data());
    Vector6d error;

    auto desired_position_cur = arm.desired_position;
    error.head(3) << current_position - desired_position_cur;
    if (arm.desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
      current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond rot_error(
        current_orientation * arm.desired_orientation.inverse());
    Eigen::AngleAxisd rot_error_aa(rot_error);
    error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
    Vector7d tau_task, tau_nullspace, tau_d;
    tau_task.setZero();
    tau_nullspace.setZero();
    tau_d.setZero();
    tau_task << jacobian.transpose() * (-arm.stiffness*error - arm.damping*(jacobian*qD));

    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                        jacobian.transpose() * jacobian_transpose_pinv) *
                           (arm.n_stiffness * (arm.desired_qn - q) -
                            (2.0 * sqrt(arm.n_stiffness)) * qD);

    tau_d <<  tau_task + coriolis + tau_nullspace;
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[k].set_value(tau_d(i));
      k++;
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn MultiCartesianImpedanceController::on_init() {
  try {
    rclcpp::Parameter arm_count;
    bool bHas_arm_count = get_node()->get_parameter("arm_count", arm_count);
    if(!bHas_arm_count){
      auto_declare<int>("arm_count", 0);
    }
    sub_desired_cartesian_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/multi_cartesian_impedance/pose_desired", 1,
      std::bind(&MultiCartesianImpedanceController::desiredCartesianCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiCartesianImpedanceController::on_configure(
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

    if(!get_node()->has_parameter(prefix + "pos_stiff")){
        get_node()->declare_parameter<double>(prefix + "pos_stiff", 100.0);
    }
    if(!get_node()->has_parameter(prefix + "rot_stiff")){
        get_node()->declare_parameter<double>(prefix + "rot_stiff", 10.0);
    }

    arm.pos_stiff = get_node()->get_parameter(prefix + "pos_stiff").as_double();
    arm.rot_stiff = get_node()->get_parameter(prefix + "rot_stiff").as_double();
    i++;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    arm.desired = Matrix4d(arm.franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
    arm.desired_position = Vector3d(arm.desired.block<3,1>(0,3));
    arm.desired_orientation = Quaterniond(arm.desired.block<3,3>(0,0));
    arm.desired_qn = Vector7d(arm.franka_robot_model_->getRobotState()->q.data());

    arm.stiffness.setIdentity();
    arm.stiffness.topLeftCorner(3, 3) << arm.pos_stiff * Matrix3d::Identity();
    arm.stiffness.bottomRightCorner(3, 3) << arm.rot_stiff * Matrix3d::Identity();
    arm.damping.setIdentity();
    arm.damping.topLeftCorner(3,3) << 2 * sqrt(arm.pos_stiff) * Matrix3d::Identity();
    arm.damping.bottomRightCorner(3, 3) << 0.8 * 2 * sqrt(arm.rot_stiff) * Matrix3d::Identity();
    arm.n_stiffness = 10.0;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/){
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.franka_robot_model_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

void MultiCartesianImpedanceController::desiredCartesianCallback(
  const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data.size() >= num_robots * 12) {
    if (msg.data[0]) {
      int offset = 0;
      for (auto& arm_container_pair : arms_) {
        auto &arm = arm_container_pair.second;
        for (auto i = 0; i < 3; ++i) {
          arm.desired_position[i] = msg.data[offset + i];
        }
        if (msg.data[offset + 11]) {
          Eigen::Matrix3d desired_orientation_mat;
          for (auto i = 0; i < 3; ++i) {
            for (auto j = 0; j < 3; ++j) {
              desired_orientation_mat(i, j) = msg.data[offset + 3 + 3*i + j];
            }
          }
          arm.desired_orientation = Eigen::Quaterniond(desired_orientation_mat);
        }
        offset += 12;
      }
    }
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MultiCartesianImpedanceController,
                       controller_interface::ControllerInterface)
