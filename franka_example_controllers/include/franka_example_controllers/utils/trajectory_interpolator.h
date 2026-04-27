/*
    Linear Trajectory Interpolator for dual-arm system
    Adapted for franka_example_controllers package
*/
#ifndef FRANKA_EXAMPLE_CONTROLLERS_TRAJECTORY_INTERPOLATOR_H
#define FRANKA_EXAMPLE_CONTROLLERS_TRAJECTORY_INTERPOLATOR_H

#include <Eigen/Dense>
#include <vector>

namespace franka_example_controllers {

using Vector14d = Eigen::Matrix<double, 14, 1>;

/**
 * Linear interpolator for 14-DOF dual-arm trajectory waypoints
 * Provides constant velocity interpolation between waypoints
 */
class LinearInterpolator14d {
public:
    LinearInterpolator14d() = delete;
    explicit LinearInterpolator14d(double time_step);

    // Configuration
    void setVelocity(const Vector14d& vel);           // Set different velocities for each joint
    void setVelocity(double vel);                       // Set same interpolation velocity for all joints
    void setInterpolationTimeStep(double time_step);    // Set interpolation time step

    // Trajectory management
    void resetTraj(const std::vector<Vector14d>& traj_wpts);
    void clear();

    // Interpolation step
    Vector14d nextStep();

    // State queries
    bool isTargetReached() const { return target_reached_; }
    bool isEmpty() const { return traj_wpts_.empty(); }

private:
    bool target_reached_;
    int n_wpts_;
    int index_;
    double dt_;            // Interpolation time step
    double alpha_;         // Current interpolation parameter
    double alpha_inc_;     // Interpolation parameter increment
    Vector14d vel_;        // Interpolation velocity
    std::vector<Vector14d> traj_wpts_;
};

} // namespace franka_example_controllers

#endif // FRANKA_EXAMPLE_CONTROLLERS_TRAJECTORY_INTERPOLATOR_H