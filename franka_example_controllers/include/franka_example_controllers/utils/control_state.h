/*
    Control State Machine for DualArmMprcController
    Adapted from dual_arm_safe_controller_sim
*/
#ifndef FRANKA_EXAMPLE_CONTROLLERS_CONTROL_STATE_H
#define FRANKA_EXAMPLE_CONTROLLERS_CONTROL_STATE_H

#include <string>

namespace franka_example_controllers {

/**
 * ControlState defines the operating modes of the controller
 */
enum class ControlState {
    TRACKING,  // Normal trajectory tracking mode
    REACTING,  // Reaction mode - trajectory stops updating but controller remains active
    STOPPING   // Stop mode - waiting for next task or exception handling
};

/**
 * Convert ControlState to string for logging
 */
inline std::string controlStateToString(const ControlState& state) {
    switch (state) {
        case ControlState::TRACKING:
            return "Control State: TRACKING";
        case ControlState::REACTING:
            return "Control State: REACTING";
        case ControlState::STOPPING:
            return "Control State: STOPPING";
        default:
            return "Control State: UNKNOWN";
    }
}

/**
 * ExceptionType defines different types of exceptions that can occur
 */
enum class ExceptionType {
    NO_EXCEPTION,          // No error
    EMPTY_TRAJECTORY,       // Trajectory interpolator is empty
    EMPTY_BUFFER,          // Trajectory buffer is empty
    LARGE_DEVIATION,       // Current configuration deviates too much from reference
    QP_SOLVER_ERROR,       // QP solver failed to find solution
    CONSTRAINT_VIOLATION    // Safety constraints violated
};

/**
 * Convert ExceptionType to string for logging
 */
inline std::string exceptionTypeToString(const ExceptionType& type) {
    switch (type) {
        case ExceptionType::NO_EXCEPTION:
            return "ExceptionType: No Exception";
        case ExceptionType::EMPTY_TRAJECTORY:
            return "ExceptionType: Empty Trajectory";
        case ExceptionType::EMPTY_BUFFER:
            return "ExceptionType: Empty Buffer";
        case ExceptionType::LARGE_DEVIATION:
            return "ExceptionType: Large Deviation";
        case ExceptionType::QP_SOLVER_ERROR:
            return "ExceptionType: QP Solver Error";
        case ExceptionType::CONSTRAINT_VIOLATION:
            return "ExceptionType: Constraint Violation";
        default:
            return "ExceptionType: Unknown";
    }
}

/**
 * ControllerType defines the underlying control algorithm
 */
enum class ControllerType {
    HQP,            // Hierarchical Quadratic Programming
    NullSpace,      // Null-space control (current implementation)
    HardSoftQP      // Hard/Soft constraint QP
};

} // namespace franka_example_controllers

#endif // FRANKA_EXAMPLE_CONTROLLERS_CONTROL_STATE_H