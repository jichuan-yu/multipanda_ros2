/*
    Trajectory Buffer for storing trajectory points
    Adapted for franka_example_controllers package
*/
#ifndef FRANKA_EXAMPLE_CONTROLLERS_TRAJECTORY_BUFFER_H
#define FRANKA_EXAMPLE_CONTROLLERS_TRAJECTORY_BUFFER_H

#include <vector>
#include <Eigen/Dense>

namespace franka_example_controllers {

using Vector14d = Eigen::Matrix<double, 14, 1>;

/**
 * Circular buffer for storing dual-arm configuration space (14 DoF) trajectory points
 * Provides filtering capabilities for smooth trajectory execution
 */
class TrajectoryBuffer14d {
public:
    TrajectoryBuffer14d() = delete;
    explicit TrajectoryBuffer14d(int buffer_size);

    // Filter settings
    void setFilter(int window_size);

    // Input/Output operations
    bool input(const Vector14d& traj_point);      // Input 1 trajectory point
    Vector14d output();                           // Output 1 traj point and move pointer forward
    Vector14d preview();                          // Output 1 traj point but don't move pointer
    Vector14d output_filter();                    // Output filtered traj point (moving average) and move pointer
    Vector14d preview_filter();                   // Output filtered traj point but don't move pointer

    // State queries
    bool isFull() const;
    bool isEmpty() const;
    bool isFilled() const;  // Check if buffer has at least filter_window_size_ + 1 points
    int validSize() const;

    // Reset
    void clear();

private:
    std::vector<Vector14d> traj_buffer_;
    const int buffer_size_;
    int filter_window_size_ = 1;  // Moving average filter window size
    int head_ = 0;
    int tail_ = 0;
};

} // namespace franka_example_controllers

#endif // FRANKA_EXAMPLE_CONTROLLERS_TRAJECTORY_BUFFER_H