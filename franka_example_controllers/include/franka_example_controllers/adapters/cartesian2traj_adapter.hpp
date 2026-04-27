#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Dense>
#include <memory>
#include <mutex>

#include "franka_example_controllers/utils/robot_kinematics.hpp"

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector14d = Eigen::Matrix<double, 14, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Quaterniond = Eigen::Quaterniond;

namespace franka_example_controllers {

/**
 * @brief Cartesian to Trajectory Adapter Node
 *
 * This adapter node converts Cartesian pose commands from key_safe_pub.py
 * into joint trajectory messages expected by the senior's dualarm_mprc controller.
 *
 * Input:  /dualarm_mprc/pose_desired (Float64MultiArray, 24 elements)
 *         Layout: [left_pos(3), left_rot(9), right_pos(3), right_rot(9)]
 *
 * Output: /dualArm_traj (JointTrajectory, 14 joints)
 *         Layout: [panda_1_joint1...7, panda_2_joint1...7]
 */
class Cartesian2TrajectoryAdapter : public rclcpp::Node {
 public:
  Cartesian2TrajectoryAdapter();
  ~Cartesian2TrajectoryAdapter() = default;

 private:
  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  // Robot models for IK
  std::shared_ptr<PandaRobot> robot1_;
  std::shared_ptr<PandaRobot> robot2_;

  // Current joint states (for IK)
  Vector7d q1_current_;
  Vector7d q2_current_;

  // Joint limits
  const Vector7d q_min_{
    (Vector7d() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished()};
  const Vector7d q_max_{
    (Vector7d() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished()};

  // Threshold for publishing new trajectory
  double position_threshold_{0.001};  // 1mm
  double rotation_threshold_{0.01};   // ~0.57 degrees

  // Last published target (for thresholding)
  Vector3d last_left_pos_;
  Matrix3d last_left_rot_;
  Vector3d last_right_pos_;
  Matrix3d last_right_rot_;

  bool has_last_target_{false};

  // Mutex for thread safety
  std::mutex target_mutex_;

  // Callback
  void cartesianCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // IK solving
  Vector7d solveIK(const Vector3d& target_pos,
                   const Matrix3d& target_rot,
                   const Vector7d& q_current,
                   const std::shared_ptr<PandaRobot>& robot);

  // Helper functions
  Matrix4d poseVectorToMatrix(const Vector3d& pos, const Matrix3d& rot) const;
  bool shouldPublishNewTarget(const Vector3d& pos, const Matrix3d& rot,
                              const Vector3d& last_pos, const Matrix3d& last_rot) const;
  void initializeRobotModels();
};

}  // namespace franka_example_controllers
