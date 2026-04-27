#include "franka_example_controllers/utils/trajectory_buffer.h"
#include <iostream>

namespace franka_example_controllers {

TrajectoryBuffer14d::TrajectoryBuffer14d(int buffer_size)
    : buffer_size_(buffer_size)
{
    traj_buffer_.resize(buffer_size_);
}

void TrajectoryBuffer14d::setFilter(int window_size)
{
    if (window_size >= buffer_size_) {
        std::cerr << "TrajectoryBuffer: Filter window size must be less than buffer size." << std::endl;
        return;
    }
    filter_window_size_ = window_size;
}

bool TrajectoryBuffer14d::input(const Vector14d& traj_point)
{
    if ((head_ + 1) % buffer_size_ == tail_) {
        return false;  // Buffer is full
    }

    traj_buffer_[head_] = traj_point;
    head_ = (head_ + 1) % buffer_size_;
    return true;
}

Vector14d TrajectoryBuffer14d::output()
{
    if (tail_ == head_) {
        return Vector14d::Zero();  // Buffer is empty
    }

    Vector14d output = traj_buffer_[tail_];
    tail_ = (tail_ + 1) % buffer_size_;
    return output;
}

Vector14d TrajectoryBuffer14d::preview()
{
    if (tail_ == head_) {
        return Vector14d::Zero();  // Buffer is empty
    }
    return traj_buffer_[tail_];
}

Vector14d TrajectoryBuffer14d::output_filter()
{
    if (!isFilled()) {
        return output();  // Not enough points for filtering
    }

    // Moving average filter
    Vector14d filtered = Vector14d::Zero();
    int count = 0;

    for (int i = 0; i < filter_window_size_ + 1; i++) {
        int idx = (tail_ + i) % buffer_size_;
        filtered += traj_buffer_[idx];
        count++;
    }

    filtered /= count;
    tail_ = (tail_ + 1) % buffer_size_;
    return filtered;
}

Vector14d TrajectoryBuffer14d::preview_filter()
{
    if (!isFilled()) {
        return preview();  // Not enough points for filtering
    }

    // Moving average filter
    Vector14d filtered = Vector14d::Zero();
    int count = 0;

    for (int i = 0; i < filter_window_size_ + 1; i++) {
        int idx = (tail_ + i) % buffer_size_;
        filtered += traj_buffer_[idx];
        count++;
    }

    filtered /= count;
    return filtered;
}

bool TrajectoryBuffer14d::isFull() const
{
    return (head_ + 1) % buffer_size_ == tail_;
}

bool TrajectoryBuffer14d::isEmpty() const
{
    return head_ == tail_;
}

bool TrajectoryBuffer14d::isFilled() const
{
    if (isEmpty()) {
        return false;
    }

    int size = (head_ - tail_ + buffer_size_) % buffer_size_;
    return size >= filter_window_size_ + 1;
}

int TrajectoryBuffer14d::validSize() const
{
    if (isEmpty()) {
        return 0;
    }
    return (head_ - tail_ + buffer_size_) % buffer_size_;
}

void TrajectoryBuffer14d::clear()
{
    head_ = 0;
    tail_ = 0;
}

} // namespace franka_example_controllers