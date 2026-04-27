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
#include "franka_example_controllers/utils/robot_kinematics.hpp"
#include "franka_example_controllers/utils/collision_env.h"
#include "franka_example_controllers/utils/hierarchical_qp.h"
#include "franka_example_controllers/utils/constraint_manager.h"
#include "franka_example_controllers/utils/control_state.h"
#include "franka_example_controllers/utils/trajectory_buffer.h"
#include "franka_example_controllers/utils/trajectory_interpolator.h"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "dual_arm_reactive_control/msg/collision_object.hpp"

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
 * Enhanced middle-layer controller that bridges key_safe_pub.py and
 * MultiJointImpedanceController with HQP-based safety control.
 *
 * Subscribes : /dualarm_mprc/pose_desired  (Float64MultiArray, 24 elements)
 *              layout: [left_pos(3), left_rotmat(9), right_pos(3), right_rotmat(9)]
 *
 * Publishes  : /dual_joint_impedance/joints_desired (Float64MultiArray, 14 elements)
 *              layout: [mj_left_q(7), mj_right_q(7)]
 *
 * Safety:  Uses Hierarchical Quadratic Programming (HQP) with ConstraintManager
 *          for advanced safety control including collision avoidance, joint limits,
 *          and constraint prioritization.
 *
 * Features:
 *   - HQP-based optimization with constraint prioritization
 *   - State machine (TRACKING/REACTING/STOPPING)
 *   - Trajectory buffering and interpolation
 *   - Advanced collision avoidance with multiple CBF constraints
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

    std::shared_ptr<PandaRobot> panda_robot_model_;
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

  // Subscriber: dynamic obstacles from environment
  rclcpp::Subscription<dual_arm_reactive_control::msg::CollisionObject>::SharedPtr sub_dynamic_obstacle_;

  // Collision environment for static and dynamic obstacles
  std::shared_ptr<CollisionEnv> collision_env_;

  // ── HQP Safety Control System ─────────────────────────────────────────────
  // HQP solver for hierarchical optimization
  std::unique_ptr<HQP::HierarchicalQP> hqp_solver_;

  // Constraint manager for safety constraints
  std::unique_ptr<ConstraintManager> constraint_manager_;

  // State machine for control modes
  ControlState current_control_state_{ControlState::STOPPING};
  ControlState next_control_state_{ControlState::STOPPING};
  ExceptionType current_exception_{ExceptionType::NO_EXCEPTION};

  // Trajectory processing system
  std::unique_ptr<TrajectoryBuffer14d> trajectory_buffer_;
  std::unique_ptr<LinearInterpolator14d> trajectory_interpolator_;

  // Control parameters
  ControllerType controller_type_{ControllerType::NullSpace};  // Start with null-space for compatibility
  bool use_hqp_{false};  // Flag to enable/disable HQP mode

  // CBF Safety Parameters (for backward compatibility)
  double cbf_gamma_{0.1};          // CBF correction gain
  double collision_d_min_{0.05};   // Minimum safe distance (m)

  // Joint limit hard constraints
  const Vector7d q_max_{
    (Vector7d() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished()};
  const Vector7d q_min_{
    (Vector7d() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished()};

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
  void dynamicObstacleCallback(const dual_arm_reactive_control::msg::CollisionObject& msg);

  // ── Safety helpers ────────────────────────────────────────────────────────
  /**
   * Build the 17-dim whole-body state vector Q = [0,0,0, q_right(7), q_left(7)].
   * (Base is fixed; the three leading zeros represent mobile-base DOFs.)
   */
  Eigen::VectorXd buildQvector(const Vector7d& q_left, const Vector7d& q_right) const;

  // ── HQP Control helpers ─────────────────────────────────────────────────────
  /**
   * Initialize HQP solver and constraint manager
   */
  bool initializeHQPSolver();

  /**
   * Solve HQP optimization problem
   */
  bool solveHQP(const Vector14d& q_current, const Vector14d& q_desired, Vector14d& dq_solution);

  /**
   * Update control state machine
   */
  void updateControlState(const Vector14d& q_current, const Vector14d& q_desired);

  /**
   * Check for state transitions and exceptions
   */
  void checkStateTransitions(const Vector14d& q_current, const Vector14d& q_desired);
};

}  // namespace franka_example_controllers
