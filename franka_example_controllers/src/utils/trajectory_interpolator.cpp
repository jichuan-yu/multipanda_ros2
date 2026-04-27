#include "franka_example_controllers/utils/trajectory_interpolator.h"
#include <cmath>
#include <iostream>

namespace franka_example_controllers {

LinearInterpolator14d::LinearInterpolator14d(double time_step)
    : target_reached_(true), n_wpts_(0), index_(0), dt_(time_step),
      alpha_(0.0)
{
    setVelocity(0.2);  // Default velocity
}

void LinearInterpolator14d::setVelocity(const Vector14d& vel)
{
    vel_ = vel;
}

void LinearInterpolator14d::setVelocity(double vel)
{
    vel_.setConstant(vel);
}

void LinearInterpolator14d::setInterpolationTimeStep(double time_step)
{
    dt_ = time_step;
}

void LinearInterpolator14d::resetTraj(const std::vector<Vector14d>& traj_wpts)
{
    if (traj_wpts.empty()) {
        std::cerr << "LinearInterpolator: Empty trajectory, ignoring." << std::endl;
        return;
    }

    traj_wpts_ = traj_wpts;
    n_wpts_ = traj_wpts_.size() - 1;
    index_ = 0;
    alpha_ = 0.0;
    target_reached_ = false;

    // Calculate alpha increment for constant velocity interpolation
    if (n_wpts_ > 0) {
        Vector14d diff = traj_wpts_[1] - traj_wpts_[0];
        double max_diff = diff.cwiseAbs().maxCoeff();

        if (max_diff > 1e-6) {
            double time_needed = max_diff / vel_.cwiseAbs().maxCoeff();
            alpha_inc_ = dt_ / time_needed;
        } else {
            alpha_inc_ = 1.0;  // Waypoints are very close
        }
    }
}

void LinearInterpolator14d::clear()
{
    traj_wpts_.clear();
    target_reached_ = true;
    n_wpts_ = 0;
    index_ = 0;
    alpha_ = 0.0;
}

Vector14d LinearInterpolator14d::nextStep()
{
    if (target_reached_) {
        return Vector14d::Zero();
    }

    Vector14d current_pos = (1.0 - alpha_) * traj_wpts_[index_] + alpha_ * traj_wpts_[index_ + 1];
    alpha_ += alpha_inc_;

    if (alpha_ >= 1.0) {
        alpha_ = 0.0;
        index_++;

        if (index_ >= n_wpts_) {
            target_reached_ = true;
            index_ = n_wpts_ > 0 ? n_wpts_ - 1 : 0;
        }
    }

    return current_pos;
}

} // namespace franka_example_controllers