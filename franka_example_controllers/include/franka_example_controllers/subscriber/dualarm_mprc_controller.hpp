#pragma once

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

/**
 * DualArmMprcController
 *
 * Middle-layer controller that bridges key_safe_pub.py and
 * MultiJointImpedanceController.
 *
 * Subscribes : /dualarm_mprc/pose_desired  (Float64MultiArray, 24 elements)
 *              layout: [left_pos(3), left_rotmat(9), right_pos(3), right_rotmat(9)]
 *
 * Publishes  : /dual_joint_impedance/joints_desired (Float64MultiArray, 14 elements)
 *              layout: [mj_left_q(7), mj_right_q(7)]
 *
 * Safety:  collision-avoidance & joint-limit gradients from redundancy_resolution
 *          are applied in the null-space of the Jacobian.
 */
class DualArmMprcController : public controller_interface::ControllerInterface {
 public:
  // ── Per-arm state ─────────────────────────────────────────────────────────
  struct ArmContainer {
    std::string arm_id_;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

    // Desired (updated from subscriber callback)
    Vector3d   desired_position{Vector3d::Zero()};
    Quaterniond desired_orientation{Quaterniond::Identity()};

    // Computed desired joint angles sent to MultiJointImpedanceController
    Vector7d q_desired{Vector7d::Zero()};

    double pos_stiff{100.0};   // Cartesian position stiffness [N/m]
    double rot_stiff{10.0};    // Cartesian rotation stiffness [Nm/rad]
  };

  // ── ControllerInterface overrides ─────────────────────────────────────────
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // ── Topology ──────────────────────────────────────────────────────────────
  int num_robots_{0};
  static constexpr int kNumJoints = 7;

  // Map key = arm_id string; std::map is sorted lexicographically.
  // With mj_left < mj_right, left is index-0 (arm_1) and right is index-1 (arm_2).
  std::map<std::string, ArmContainer> arms_;

  // ── ROS communication ─────────────────────────────────────────────────────
  // Subscriber: receives Cartesian target from key_safe_pub
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pose_desired_;

  // Publisher: sends joint targets to MultiJointImpedanceController
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_desired_;

  // Publisher: collision markers for visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_collision_markers_;

  // ── Thread-safety ─────────────────────────────────────────────────────────
  std::mutex target_mutex_;
  bool has_target_{false};   // Have we received at least one goal?

  // ── Avoidance cycle counter (compute gradients every N cycles) ─────────────
  int avoidance_counter_{0};
  static constexpr int kAvoidanceInterval = 50;  // call gradient every 50 ms @ 1 kHz

  // Per-arm cached gradients — updated every kAvoidanceInterval
  std::array<Vector7d, 2> grad_coll_{};
  std::array<Vector7d, 2> grad_qlim_{};

  // ── Callback ──────────────────────────────────────────────────────────────
  void desiredPoseCallback(const std_msgs::msg::Float64MultiArray& msg);

  // ── Safety helpers ────────────────────────────────────────────────────────
  /**
   * Build the 17-dim whole-body state vector Q = [0,0,0, q_right(7), q_left(7)].
   * (Base is fixed; the three leading zeros represent mobile-base DOFs.)
   */
  Eigen::VectorXd buildQvector(const Vector7d& q_left, const Vector7d& q_right) const;
};

}  // namespace franka_example_controllers
