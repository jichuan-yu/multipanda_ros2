/**
 * dualarm_mprc_controller.cpp
 *
 * Middle-layer safety controller between key_safe_pub and
 * MultiJointImpedanceController.
 *
 * Input  : /dualarm_mprc/pose_desired  (Float64MultiArray 24-dim)
 * Output : /dual_joint_impedance/joints_desired (Float64MultiArray 14-dim)
 */

#include <franka_example_controllers/subscriber/dualarm_mprc_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Dense>
#include <multi_mode_controller/utils/redundancy_resolution.h>

#include "pluginlib/class_list_macros.hpp"

namespace franka_example_controllers {

// ─────────────────────────────────────────────────────────────────────────────
// Damped pseudo-inverse  J† = Jᵀ (J Jᵀ + λ² I)⁻¹
// ─────────────────────────────────────────────────────────────────────────────
static void dampedPseudoInverse(const Eigen::MatrixXd& J,
                                Eigen::MatrixXd&       J_pinv,
                                double lambda = 0.05) {
  const int m = J.rows();
  Eigen::MatrixXd JJt = J * J.transpose();
  J_pinv = J.transpose() * (JJt + lambda * lambda * Eigen::MatrixXd::Identity(m, m)).inverse();
}

// ─────────────────────────────────────────────────────────────────────────────
// command_interface_configuration
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::InterfaceConfiguration
DualArmMprcController::command_interface_configuration() const {
  // This controller does NOT directly drive hardware joints.
  // It only publishes to /dual_joint_impedance/joints_desired.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

// ─────────────────────────────────────────────────────────────────────────────
// state_interface_configuration
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::InterfaceConfiguration
DualArmMprcController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& arm_pair : arms_) {
    for (const auto& name :
         arm_pair.second.franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  return config;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_init
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn DualArmMprcController::on_init() {
  try {
    auto_declare<int>("arm_count", 2);

    // Subscriber: Cartesian target poses from key_safe_pub
    sub_pose_desired_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/dualarm_mprc/pose_desired", 1,
        std::bind(&DualArmMprcController::desiredPoseCallback, this,
                  std::placeholders::_1));

    // Publisher: joint targets to MultiJointImpedanceController
    pub_joint_desired_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/dual_joint_impedance/joints_desired", 1);

    // Publisher: collision markers
    pub_collision_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mprc/collision_spheres", 1);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "DualArmMprcController::on_init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn DualArmMprcController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!get_node()->get_parameter("arm_count", num_robots_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "arm_count parameter not found");
    return CallbackReturn::FAILURE;
  }

  arms_.clear();
  for (int i = 1; i <= num_robots_; i++) {
    std::string prefix = "arm_" + std::to_string(i);
    std::string id_param = prefix + ".arm_id";
    if (!get_node()->has_parameter(id_param)) {
      get_node()->declare_parameter<std::string>(id_param, "");
    }
    std::string arm_id = get_node()->get_parameter(id_param).as_string();
    
    if (arm_id.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "arm_%d.arm_id is empty", i);
      return CallbackReturn::FAILURE;
    }

    auto& arm = arms_[arm_id];
    arm.arm_id_ = arm_id;

    arm.franka_robot_model_ =
        std::make_unique<franka_semantic_components::FrankaRobotModel>(
            franka_semantic_components::FrankaRobotModel(arm.arm_id_ + "/robot_model",
                                                         arm.arm_id_));

    if (!get_node()->has_parameter(prefix + ".pos_stiff")) {
      get_node()->declare_parameter<double>(prefix + ".pos_stiff", 400.0);
    }
    if (!get_node()->has_parameter(prefix + ".rot_stiff")) {
      get_node()->declare_parameter<double>(prefix + ".rot_stiff", 15.0);
    }
    arm.pos_stiff = get_node()->get_parameter(prefix + ".pos_stiff").as_double();
    arm.rot_stiff = get_node()->get_parameter(prefix + ".rot_stiff").as_double();
  }

  if (static_cast<int>(arms_.size()) != num_robots_) {
    RCLCPP_FATAL(get_node()->get_logger(), "Duplicate arm IDs or size mismatch");
    return CallbackReturn::FAILURE;
  }

  // Initialise cached gradients to zero
  for (auto& g : grad_coll_) g.setZero();
  for (auto& g : grad_qlim_) g.setZero();

  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn DualArmMprcController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto& arm_pair : arms_) {
    auto& arm = arm_pair.second;
    arm.franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    // Initialise desired pose to current EE pose
    Eigen::Map<const Matrix4d> T(
        arm.franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
    arm.desired_position    = T.block<3, 1>(0, 3);
    arm.desired_orientation = Eigen::Quaterniond(T.block<3, 3>(0, 0));

    // Initialise q_desired to current joint angles
    arm.q_desired = Vector7d(
        arm.franka_robot_model_->getRobotState()->q.data());
  }

  avoidance_counter_ = 0;
  has_target_ = false;
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn DualArmMprcController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto& arm_pair : arms_) {
    arm_pair.second.franka_robot_model_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// desiredPoseCallback  — parse 24-dim msg into per-arm targets
// ─────────────────────────────────────────────────────────────────────────────
void DualArmMprcController::desiredPoseCallback(
    const std_msgs::msg::Float64MultiArray& msg) {
  if (static_cast<int>(msg.data.size()) < num_robots_ * 12) {
    return;
  }
  // Skip messages whose first element is zero (legacy guard from key_pub.py)
  if (msg.data[0] == 0.0) {
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  int offset = 0;
  for (auto& arm_pair : arms_) {
    auto& arm = arm_pair.second;

    // Position
    arm.desired_position = Vector3d(msg.data[offset + 0],
                                    msg.data[offset + 1],
                                    msg.data[offset + 2]);

    // Rotation matrix (row-major in msg, same as key_pub.py R.flatten())
    if (msg.data[offset + 11] != 0.0) {   // non-zero marker check
      Eigen::Matrix3d R;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          R(r, c) = msg.data[offset + 3 + 3 * r + c];
        }
      }
      arm.desired_orientation = Eigen::Quaterniond(R);
      arm.desired_orientation.normalize();
    }
    offset += 12;
  }
  has_target_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildQvector
// Layout: [0, 0, 0, q_right(7), q_left(7)]  ← redundancy_resolution convention
// The map is sorted: mj_left (index 0) < mj_right (index 1)
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXd DualArmMprcController::buildQvector(
    const Vector7d& q_left,
    const Vector7d& q_right) const {
  Eigen::VectorXd Q(17);
  Q.setZero();
  Q.segment<7>(3)  = q_right;   // right arm  (isRightArm == true  → index 3..9)
  Q.segment<7>(10) = q_left;    // left arm   (isRightArm == false → index 10..16)
  return Q;
}

// ─────────────────────────────────────────────────────────────────────────────
// update — main control loop
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::return_type DualArmMprcController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  if (!has_target_) {
    return controller_interface::return_type::OK;
  }

  // ── Collect current joint states for both arms ───────────────────────────
  // Map iterates lexicographically: mj_left (arm[0]), mj_right (arm[1])
  std::vector<ArmContainer*> arm_ptrs;
  for (auto& pair : arms_) {
    arm_ptrs.push_back(&pair.second);
  }

  // arm_ptrs[0] = mj_left (arm_1), arm_ptrs[1] = mj_right (arm_2)
  ArmContainer& left_arm  = *arm_ptrs[0];
  ArmContainer& right_arm = *arm_ptrs[1];

  Vector7d q_left  = Vector7d(left_arm.franka_robot_model_->getRobotState()->q.data());
  Vector7d q_right = Vector7d(right_arm.franka_robot_model_->getRobotState()->q.data());

  // ── Update collision/joint-limit gradients every kAvoidanceInterval ───────
  ++avoidance_counter_;
  if (avoidance_counter_ >= kAvoidanceInterval) {
    avoidance_counter_ = 0;
    Eigen::VectorXd Q = buildQvector(q_left, q_right);

    // Left arm: isRightArm = false
    grad_coll_[0] = redundancy_resolution::CollisionAvoidanceGradient(Q, false);
    grad_qlim_[0] = redundancy_resolution::JointLimitPotentialGradient(Q, false);

    // Right arm: isRightArm = true
    grad_coll_[1] = redundancy_resolution::CollisionAvoidanceGradient(Q, true);
    grad_qlim_[1] = redundancy_resolution::JointLimitPotentialGradient(Q, true);
  }

  // ── Per-arm Cartesian → joint-space computation ──────────────────────────
  std::vector<Vector7d> q_desired_list(2);

  for (int arm_idx = 0; arm_idx < 2; ++arm_idx) {
    ArmContainer& arm = *arm_ptrs[arm_idx];
    const Vector7d& q_cur = (arm_idx == 0) ? q_left : q_right;

    // Current EE pose from franka_robot_model
    Eigen::Map<const Matrix4d> T_cur(
        arm.franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
    Vector3d    current_pos = T_cur.block<3, 1>(0, 3);
    Eigen::Quaterniond current_ori(T_cur.block<3, 3>(0, 0));

    // ── Copy desired pose (thread-safe) ────────────────────────────────────
    Vector3d     target_pos;
    Eigen::Quaterniond target_ori;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target_pos = arm.desired_position;
      target_ori = arm.desired_orientation;
    }

    // ── Cartesian error (6-dim: position + orientation) ────────────────────
    Vector6d delta_x;
    delta_x.head<3>() = target_pos - current_pos;

    // Orientation error as axis-angle via quaternion difference
    Eigen::Quaterniond q_err = target_ori * current_ori.inverse();
    q_err.normalize();
    Eigen::AngleAxisd aa(q_err);
    delta_x.tail<3>() = aa.axis() * aa.angle();

    // ── Jacobian (6 × 7, from franka_robot_model) ─────────────────────────
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(
        arm.franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());

    // ── Damped pseudo-inverse ──────────────────────────────────────────────
    Eigen::MatrixXd J_pinv;
    dampedPseudoInverse(J, J_pinv);

    // ── Task-space contribution: delta_q = J† * gain * delta_x ───────────
    const double k_task = 1.0;   // proportional gain
    Vector7d delta_q_task = k_task * J_pinv * delta_x;

    // ── Null-space safety contribution ─────────────────────────────────────
    //    N = I - J† J  (null-space projector)
    //    delta_q_null = N * (-grad_coll - grad_qlim)
    Eigen::Matrix<double, 7, 7> N =
        Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;
    const double k_null = 0.5;
    Vector7d safe_gradient = -(grad_coll_[arm_idx] + grad_qlim_[arm_idx]);
    Vector7d delta_q_null = k_null * N * safe_gradient;

    // ── Integrate: q_desired = q_current + delta_q ─────────────────────────
    q_desired_list[arm_idx] = q_cur + delta_q_task + delta_q_null;

    // ── Joint-limit saturation ─────────────────────────────────────────────
    const static Vector7d q_max =
        (Vector7d() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished();
    const static Vector7d q_min =
        (Vector7d() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
    q_desired_list[arm_idx] = q_desired_list[arm_idx].cwiseMax(q_min).cwiseMin(q_max);

    // Store for next cycle
    arm.q_desired = q_desired_list[arm_idx];
  }

  // ── Publish 14-dim joint target ──────────────────────────────────────────
  // Layout: [mj_left(7), mj_right(7)]  — matches dual_joint_impedance_controller
  std_msgs::msg::Float64MultiArray joint_msg;
  joint_msg.data.resize(2 * kNumJoints);

  for (int j = 0; j < kNumJoints; ++j) {
    joint_msg.data[j]              = q_desired_list[0](j);  // left
    joint_msg.data[kNumJoints + j] = q_desired_list[1](j);  // right
  }
  pub_joint_desired_->publish(joint_msg);

  // ── Visualization of Component B Collision Model ────────────────────────
  // We use redundancy_resolution's getJointsPositions to visualize the joints
  // since this controller uses that library for safety.
  visualization_msgs::msg::MarkerArray markers;
  Eigen::VectorXd Q = buildQvector(q_left, q_right);

  auto add_arm_markers = [&](bool is_right, int start_id, float r, float g, float b) {
    Eigen::MatrixXd pos = redundancy_resolution::getJointsPositions(Q, is_right);
    for (int i = 0; i < pos.cols(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = get_node()->now();
      m.ns = is_right ? "right_arm_joints" : "left_arm_joints";
      m.id = start_id + i;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pos(0, i);
      m.pose.position.y = pos(1, i);
      m.pose.position.z = pos(2, i);
      m.scale.x = m.scale.y = m.scale.z = 0.08; // Representative size for joint spheres
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.6;
      markers.markers.push_back(m);
    }
  };

  add_arm_markers(false, 0, 0.0, 1.0, 0.0); // Left Green
  add_arm_markers(true, 10, 1.0, 1.0, 0.0); // Right Yellow
  pub_collision_markers_->publish(markers);

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmMprcController,
                       controller_interface::ControllerInterface)
