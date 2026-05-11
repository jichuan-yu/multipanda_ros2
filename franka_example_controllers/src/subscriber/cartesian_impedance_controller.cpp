#include <franka_example_controllers/subscriber/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>
#include <franka/model.h>

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // should be model interface
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& /*period*/) {
  Eigen::Map<const Matrix4d> current(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  Eigen::Vector3d current_position(current.block<3,1>(0,3));
  Eigen::Quaterniond current_orientation(current.block<3,3>(0,0));
  const auto& robot_state = *franka_robot_model_->getRobotState();
  Eigen::Map<const Matrix7d> inertia(franka_robot_model_->getMassMatrix().data());
  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Eigen::Map<const Vector7d> qD(robot_state.dq.data());
  Eigen::Map<const Vector7d> q(robot_state.q.data());
  Vector6d error;

  auto desired_position_cur = desired_position;
  error.head(3) << current_position - desired_position_cur;
  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }
  Eigen::Quaterniond rot_error(
      current_orientation * desired_orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task.setZero();
  tau_nullspace.setZero();
  tau_d.setZero();
  tau_task << jacobian.transpose() * (-stiffness*error - damping*(jacobian*qD));

  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (n_stiffness * (desired_qn - q) -
                          (2.0 * sqrt(n_stiffness)) * qD);

  tau_d <<  tau_task + coriolis + tau_nullspace;

  std::array<double, 7> tau_d_calculated{};
  for (int i = 0; i < num_joints; ++i) {
    tau_d_calculated[i] = tau_d(i);
  }
  const std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_saturated[i]);
  }

  if (ee_pose_publisher_) {
    geometry_msgs::msg::PoseStamped ee_pose_msg;
    ee_pose_msg.header.stamp = time;
    ee_pose_msg.header.frame_id = arm_id_ + "_link0";
    ee_pose_msg.pose.position.x = current_position.x();
    ee_pose_msg.pose.position.y = current_position.y();
    ee_pose_msg.pose.position.z = current_position.z();
    ee_pose_msg.pose.orientation.w = current_orientation.w();
    ee_pose_msg.pose.orientation.x = current_orientation.x();
    ee_pose_msg.pose.orientation.y = current_orientation.y();
    ee_pose_msg.pose.orientation.z = current_orientation.z();
    ee_pose_publisher_->publish(ee_pose_msg);
  }

  if (external_wrench_publisher_) {
    geometry_msgs::msg::WrenchStamped external_wrench_msg;
    external_wrench_msg.header.stamp = time;
    external_wrench_msg.header.frame_id = arm_id_ + "_link0";
    external_wrench_msg.wrench.force.x = robot_state.O_F_ext_hat_K[0];
    external_wrench_msg.wrench.force.y = robot_state.O_F_ext_hat_K[1];
    external_wrench_msg.wrench.force.z = robot_state.O_F_ext_hat_K[2];
    external_wrench_msg.wrench.torque.x = robot_state.O_F_ext_hat_K[3];
    external_wrench_msg.wrench.torque.y = robot_state.O_F_ext_hat_K[4];
    external_wrench_msg.wrench.torque.z = robot_state.O_F_ext_hat_K[5];
    external_wrench_publisher_->publish(external_wrench_msg);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("pos_stiff", 100);
    auto_declare<double>("rot_stiff", 10);
    auto_declare<double>("n_stiffness", 10.0);
    sub_desired_cartesian_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_impedance/target_pose", 1,
      std::bind(&CartesianImpedanceController::desiredCartesianCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  pos_stiff = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff = get_node()->get_parameter("rot_stiff").as_double();
  n_stiffness = get_node()->get_parameter("n_stiffness").as_double();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
  ee_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_impedance/ee_pose", 1);
  external_wrench_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_impedance/external_wrench", 1);
  auto parameters = get_node()->list_parameters({}, 10);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  desired = Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  desired_position = Vector3d(desired.block<3,1>(0,3));
  desired_orientation = Quaterniond(desired.block<3,3>(0,0));
  desired_qn = Vector7d(franka_robot_model_->getRobotState()->q.data());

  stiffness.setIdentity();
  stiffness.topLeftCorner(3, 3) << pos_stiff * Matrix3d::Identity();
  stiffness.bottomRightCorner(3, 3) << rot_stiff * Matrix3d::Identity();
  // Simple critical damping
  damping.setIdentity();
  damping.topLeftCorner(3,3) << 2 * sqrt(pos_stiff) * Matrix3d::Identity();
  damping.bottomRightCorner(3, 3) << 0.8 * 2 * sqrt(rot_stiff) * Matrix3d::Identity();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/){
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::desiredCartesianCallback(
  const geometry_msgs::msg::PoseStamped& msg) {
  desired_position << msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z;
  desired_orientation = Eigen::Quaterniond(
      msg.pose.orientation.w,
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z);
}

std::array<double, 7> CartesianImpedanceController::saturateTorqueRate(
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
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
