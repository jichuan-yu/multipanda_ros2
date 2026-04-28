#include "franka_example_controllers/adapters/cartesian2traj_adapter.hpp"
#include <franka_semantic_components/franka_robot_model.hpp>

namespace franka_example_controllers {

Cartesian2TrajectoryAdapter::Cartesian2TrajectoryAdapter()
    : Node("cartesian2traj_adapter") {

  // Initialize joint states to zero
  q1_current_.setZero();
  q2_current_.setZero();

  // Initialize robot models
  initializeRobotModels();

  // Create subscriber for joint states (to get current robot configuration)
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Cartesian2TrajectoryAdapter::jointStateCallback, this,
                std::placeholders::_1));

  // Create subscriber for Cartesian commands
  cartesian_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dualarm_mprc/pose_desired", 10,
      std::bind(&Cartesian2TrajectoryAdapter::cartesianCallback, this,
                std::placeholders::_1));

  // Create publisher for joint trajectories
  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/dualArm_traj", 10);

  RCLCPP_INFO(this->get_logger(),
              "Cartesian2TrajectoryAdapter initialized");
  RCLCPP_INFO(this->get_logger(),
              "Subscribing to: /joint_states and /dualarm_mprc/pose_desired");
  RCLCPP_INFO(this->get_logger(),
              "Publishing to: /dualArm_traj");
}

void Cartesian2TrajectoryAdapter::initializeRobotModels() {
  // Initialize PandaRobot models for IK solving
  // These paths should match the actual collision sphere configuration
  std::string collision_yaml =
      "/home/xiaozy24/dual_panda_ws/src/dualarm_mprc/dualarm_reactive_control/config/panda_collision_spheres.yaml";

  robot1_ = std::make_shared<PandaRobot>(1, collision_yaml);
  robot2_ = std::make_shared<PandaRobot>(2, collision_yaml);

  // Set base positions to match MuJoCo simulation (CRITICAL!)
  // MuJoCo: mj_left at (0, 0.26, 0), mj_right at (0, -0.26, 0)
  // This MUST match both adapter and senior controller!
  robot1_->setBase(Vector3d(0, 0.26, 0), Vector3d(0, 0, 0));
  robot2_->setBase(Vector3d(0, -0.26, 0), Vector3d(0, 0, 0));

  RCLCPP_INFO(this->get_logger(),
              "Robot models initialized with collision spheres");
  RCLCPP_INFO(this->get_logger(),
              "  robot1 (panda_1) base: (0, 0.26, 0)");
  RCLCPP_INFO(this->get_logger(),
              "  robot2 (panda_2) base: (0, -0.26, 0)");
}

void Cartesian2TrajectoryAdapter::cartesianCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

  if (msg->data.size() != 24) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Expected 24 elements, got %zu", msg->data.size());
    return;
  }

  // Rate limiting: check minimum time interval
  if (has_last_target_) {
    rclcpp::Time current_time = this->now();
    double time_since_last = (current_time - last_publish_time_).seconds();
    if (time_since_last < 0.1) {  // 100ms = 10Hz max publish rate
      return;  // Skip this message to avoid flooding senior's controller
    }
  }

  // Extract left arm target
  Vector3d left_pos;
  Matrix3d left_rot;
  left_pos << msg->data[0], msg->data[1], msg->data[2];
  left_rot << msg->data[3],  msg->data[4],  msg->data[5],
             msg->data[6],  msg->data[7],  msg->data[8],
             msg->data[9],  msg->data[10], msg->data[11];

  // Extract right arm target
  Vector3d right_pos;
  Matrix3d right_rot;
  right_pos << msg->data[12], msg->data[13], msg->data[14];
  right_rot << msg->data[15], msg->data[16], msg->data[17],
              msg->data[18], msg->data[19], msg->data[20],
              msg->data[21], msg->data[22], msg->data[23];

  // Check if we should publish (threshold-based filtering)
  if (has_last_target_) {
    std::lock_guard<std::mutex> lock(target_mutex_);

    bool left_changed = shouldPublishNewTarget(left_pos, left_rot,
                                                last_left_pos_, last_left_rot_);
    bool right_changed = shouldPublishNewTarget(right_pos, right_rot,
                                                 last_right_pos_, last_right_rot_);

    if (!left_changed && !right_changed) {
      return;  // Target change too small, skip
    }
  }

  // Solve IK for both arms
  Vector7d q1_target = solveIK(left_pos, left_rot, q1_current_, robot1_);
  Vector7d q2_target = solveIK(right_pos, right_rot, q2_current_, robot2_);

  // Check IK solution validity
  if (q1_target.hasNaN() || q2_target.hasNaN()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "IK solution contains NaN, skipping this target");
    return;
  }

  // Apply joint limits
  q1_target = q1_target.cwiseMax(q_min_).cwiseMin(q_max_);
  q2_target = q2_target.cwiseMax(q_min_).cwiseMin(q_max_);

  // Safety margin for joint limits (especially for joint 4 which has special range)
  const double margin = 0.1;  // 0.1 rad margin from limits
  Vector7d q_min_safe = q_min_ + Vector7d::Constant(margin);
  Vector7d q_max_safe = q_max_ - Vector7d::Constant(margin);

  // Check if targets are too close to limits
  bool q1_too_close = ((q1_target.array() < q_min_safe.array()) ||
                       (q1_target.array() > q_max_safe.array())).any();
  bool q2_too_close = ((q2_target.array() < q_min_safe.array()) ||
                       (q2_target.array() > q_max_safe.array())).any();

  if (q1_too_close || q2_too_close) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Target position too close to joint limits, skipping");
    return;
  }

  // Note: q1_current_ and q2_current_ are updated by jointStateCallback
  // from actual robot states, not from IK results

  // Construct JointTrajectory message
  auto traj_msg = trajectory_msgs::msg::JointTrajectory();
  traj_msg.header.stamp = this->now();
  traj_msg.header.frame_id = "world";

  // Joint names (matching senior's project)
  traj_msg.joint_names = {
    "panda_1_joint1", "panda_1_joint2", "panda_1_joint3",
    "panda_1_joint4", "panda_1_joint5", "panda_1_joint6", "panda_1_joint7",
    "panda_2_joint1", "panda_2_joint2", "panda_2_joint3",
    "panda_2_joint4", "panda_2_joint5", "panda_2_joint6", "panda_2_joint7"
  };

  // Create trajectory point
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(14);

  for (int i = 0; i < 7; i++) {
    point.positions[i] = q1_target(i);
    point.positions[i + 7] = q2_target(i);
  }

  // Velocities and accelerations are not required (senior's controller
  // will generate these through trajectory interpolation)

  traj_msg.points.push_back(point);

  // Publish trajectory
  traj_pub_->publish(traj_msg);

  // Update last target
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    last_left_pos_ = left_pos;
    last_left_rot_ = left_rot;
    last_right_pos_ = right_pos;
    last_right_rot_ = right_rot;
    has_last_target_ = true;
    last_publish_time_ = this->now();  // Update last publish time
  }

  RCLCPP_INFO(this->get_logger(),
              "Published trajectory target: q1=[%.3f, %.3f, %.3f, ...], q2=[%.3f, %.3f, %.3f, ...]",
              q1_target(0), q1_target(1), q1_target(2),
              q2_target(0), q2_target(1), q2_target(2));
}

Vector7d Cartesian2TrajectoryAdapter::solveIK(
    const Vector3d& target_pos,
    const Matrix3d& target_rot,
    const Vector7d& q_current,
    const std::shared_ptr<PandaRobot>& robot) {

  // Get current end-effector pose using PandaRobot::getT
  Matrix4d T_current;
  robot->getT(q_current, T_current);

  Vector3d pos_current = T_current.block<3, 1>(0, 3);
  Matrix3d rot_current = T_current.block<3, 3>(0, 0);

  // Compute Cartesian error
  Vector3d delta_pos = target_pos - pos_current;

  Matrix3d rot_error_matrix = target_rot * rot_current.transpose();
  Quaterniond q_error(rot_error_matrix);
  q_error.normalize();
  Eigen::AngleAxisd aa(q_error);
  Vector3d rot_error_axis = aa.axis();
  double rot_error_angle = aa.angle();

  // 6-dim Cartesian error [position, orientation]
  Eigen::Matrix<double, 6, 1> delta_x;
  delta_x.head<3>() = delta_pos;
  delta_x.tail<3>() = rot_error_axis * rot_error_angle;

  // Get Jacobian using PandaRobot::getJacobian_world
  // Note: Must use MatrixXd (dynamic size) as required by the interface
  Eigen::MatrixXd J_dynamic;
  robot->getJacobian_world(q_current, J_dynamic);

  // Convert to fixed-size matrix for computation
  Eigen::Matrix<double, 6, 7> J = J_dynamic;

  // Damped pseudo-inverse
  double damping = 0.01;
  Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 7, 6> J_pinv =
      J.transpose() * (J * J.transpose() + damping * damping * I6).inverse();

  // Compute joint increment
  Vector7d delta_q = J_pinv * delta_x;

  // Limit joint increment (prevent large jumps)
  // Use conservative limits to avoid HQP constraint conflicts in senior's controller
  const double max_delta = 0.05;  // rad (reduced from 0.1 for safety)
  const double max_delta_joint4 = 0.02;  // joint 4 has special limits, be extra careful

  for (int i = 0; i < 7; ++i) {
    double limit = (i == 3) ? max_delta_joint4 : max_delta;  // Joint 4 (index 3)
    if (std::abs(delta_q(i)) > limit) {
      delta_q(i) = (delta_q(i) > 0) ? limit : -limit;
    }
  }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Joint increment: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       delta_q(0), delta_q(1), delta_q(2), delta_q(3),
                       delta_q(4), delta_q(5), delta_q(6));

  // Compute target joint position
  Vector7d q_target = q_current + delta_q;

  return q_target;
}

void Cartesian2TrajectoryAdapter::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {

  // Update current joint states from actual robot configuration
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];

    // Extract mj_left joints (panda_1)
    if (joint_name.find("mj_left_joint") == 0) {
      std::string joint_num = joint_name.substr(13);  // "mj_left_joint" length
      int joint_idx = std::stoi(joint_num) - 1;
      if (joint_idx >= 0 && joint_idx < 7) {
        q1_current_(joint_idx) = msg->position[i];
      }
    }
    // Extract mj_right joints (panda_2)
    else if (joint_name.find("mj_right_joint") == 0) {
      std::string joint_num = joint_name.substr(14);  // "mj_right_joint" length
      int joint_idx = std::stoi(joint_num) - 1;
      if (joint_idx >= 0 && joint_idx < 7) {
        q2_current_(joint_idx) = msg->position[i];
      }
    }
  }
}

Matrix4d Cartesian2TrajectoryAdapter::poseVectorToMatrix(
    const Vector3d& pos, const Matrix3d& rot) const {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = rot;
  T.block<3, 1>(0, 3) = pos;
  return T;
}

bool Cartesian2TrajectoryAdapter::shouldPublishNewTarget(
    const Vector3d& pos, const Matrix3d& rot,
    const Vector3d& last_pos, const Matrix3d& last_rot) const {

  // Check position change
  double pos_change = (pos - last_pos).norm();

  // Check orientation change
  Matrix3d rot_change = rot * last_rot.transpose();
  Quaterniond q_change(rot_change);
  double angle_change = 2.0 * std::acos(std::abs(q_change.w()));

  return (pos_change > position_threshold_) ||
         (angle_change > rotation_threshold_);
}

}  // namespace franka_example_controllers

// Main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<franka_example_controllers::Cartesian2TrajectoryAdapter>();

  RCLCPP_INFO(node->get_logger(),
              "Cartesian2TrajectoryAdapter node started, spinning...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
