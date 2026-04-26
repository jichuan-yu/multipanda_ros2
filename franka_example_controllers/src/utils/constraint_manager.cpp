#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include "franka_example_controllers/utils/constraint_manager.h"

ConstraintManager::ConstraintManager(PandaRobot &robot1, PandaRobot &robot2, CollisionEnv &collision_env,
                                     std::string control_params_config)
    : robot1_(robot1), robot2_(robot2), collision_env_(collision_env)
{
    loadControlParams(control_params_config);
}

ConstraintManager::~ConstraintManager()
{
}

std::string constraintType2String(const ConstraintType &type)
{
    switch (type)
    {
    case ConstraintType::JOINT_LIMIT:
        return "JOINT_LIMIT";
    case ConstraintType::JOINT_LIMIT_WITH_ACC:
        return "JOINT_LIMIT_WITH_ACC";
    case ConstraintType::RELATIVE_POSE:
        return "RELATIVE_POSE";
    case ConstraintType::RELATIVE_POSE_CLF:
        return "RELATIVE_POSE_CLF";
    case ConstraintType::TILT_LIMIT:
        return "TILT_LIMIT";
    case ConstraintType::COLLISION_AVOIDANCE_CBF:
        return "COLLISION_AVOIDANCE_CBF";
    case ConstraintType::COLLISION_AVOIDANCE_TVCBF:
        return "COLLISION_AVOIDANCE_TVCBF";
    case ConstraintType::COLLISION_AVOIDANCE_ALLCBF:
        return "COLLISION_AVOIDANCE_ALLCBF";
    case ConstraintType::MUTUAL_COLLISION_AVOIDANCE:
        return "MUTUAL_COLLISION_AVOIDANCE";
    case ConstraintType::SINGULARITY_AVOIDANCE:
        return "SINGULARITY_AVOIDANCE";
    default:
        return "UNKNOWN";
    }
}

void ConstraintManager::checkConstraintParams(const ConstraintType &type)
{
    switch (type)
    {
    case ConstraintType::JOINT_LIMIT:
        if (!(joint_limit_flag_ && joint_velocity_limit_flag_))
        {
            std::cout << "WARNING: Joint limit/velocity limit has not been set. " << std::endl;
        }
        break;
    case ConstraintType::JOINT_LIMIT_WITH_ACC:
        if (!(joint_limit_flag_ && joint_velocity_limit_flag_ && joint_acceleration_limit_flag_))
        {
            std::cout << "WARNING: Joint limit/velocity/acc limit has not been set. " << std::endl;
        }
        break;
    case ConstraintType::RELATIVE_POSE:
        if (!relative_pose_flag_)
        {
            std::cout << "WARNING: Relative pose has not been set. " << std::endl;
        }
        break;
    case ConstraintType::RELATIVE_POSE_CLF:
        if (!relative_pose_flag_)
        {
            std::cout << "WARNING: Relative pose has not been set. " << std::endl;
        }
        break;
    case ConstraintType::TILT_LIMIT:
        if (!tilt_limit_flag_)
        {
            std::cout << "WARNING: Tilt limit has not been set. " << std::endl;
        }
        break;
    case ConstraintType::COLLISION_AVOIDANCE_CBF:
    case ConstraintType::COLLISION_AVOIDANCE_TVCBF:
    case ConstraintType::COLLISION_AVOIDANCE_ALLCBF:
    case ConstraintType::MUTUAL_COLLISION_AVOIDANCE:
    case ConstraintType::SINGULARITY_AVOIDANCE:
        break;
    default:
        break;
    }
}

void ConstraintManager::addConstraint(const Constraint &constraint)
{
    if (constraint.priority < 0)
    {
        std::cerr << "ERROR: Priority should be non-negative, failed to add constraint." << std::endl;
        return;
    }
    for (const auto &existing_constraint : constraints_)
    {
        if (existing_constraint.type == constraint.type)
        {
            std::cout << "Warning! Constraint type: " << constraintType2String(constraint.type) << " already exists, failed to add constraint." << std::endl;
            return;
        }
    }
    std::cout << "Add constraint: " << constraintType2String(constraint.type) << ", Priority: "
              << constraint.priority << std::endl;
    constraints_.push_back(constraint);
    showConstraintList();
    checkConstraintParams(constraint.type);
}

void ConstraintManager::insertConstraint(const Constraint &constraint)
{
    if (constraint.priority < 0)
    {
        std::cerr << "ERROR: Priority should be non-negative, failed to add constraint." << std::endl;
        return;
    }
    for (const auto &existing_constraint : constraints_)
    {
        if (existing_constraint.type == constraint.type)
        {
            std::cout << "Warning! Constraint type: " << constraintType2String(constraint.type) << " already exists, failed to add constraint." << std::endl;
            return;
        }
    }
    // increment the priority of all constraints with a priority >= new constraint's priority by 1.
    for (auto &existing_constraint : constraints_)
    {
        if (existing_constraint.priority >= constraint.priority)
        {
            existing_constraint.priority++;
        }
    }
    std::cout << "Add constraint: " << constraintType2String(constraint.type) << ", Priority: "
              << constraint.priority << std::endl;
    constraints_.push_back(constraint);
    showConstraintList();
    checkConstraintParams(constraint.type);
}

void ConstraintManager::removeConstraint(const ConstraintType &type)
{
    switch (type)
    {
    case ConstraintType::JOINT_LIMIT:
    case ConstraintType::JOINT_LIMIT_WITH_ACC:
        std::cout << "WARNING! Remove joint limit constraint is dangerous! Failed to remove constraint" << std::endl;
        return;
    case ConstraintType::RELATIVE_POSE:
    case ConstraintType::RELATIVE_POSE_CLF:
        relative_pose_flag_ = false;
        break;
    case ConstraintType::TILT_LIMIT:
        tilt_limit_flag_ = false;
        break;
    case ConstraintType::COLLISION_AVOIDANCE_CBF:
    case ConstraintType::COLLISION_AVOIDANCE_TVCBF:
    case ConstraintType::COLLISION_AVOIDANCE_ALLCBF:
    case ConstraintType::MUTUAL_COLLISION_AVOIDANCE:
    case ConstraintType::SINGULARITY_AVOIDANCE:
        break;
    default:
        break;
    }

    for (auto it = constraints_.begin(); it != constraints_.end();)
    {
        if (it->type == type)
        {
            it = constraints_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    std::cout << "Remove constraint: " << constraintType2String(type) << std::endl;
    showConstraintList();
}

void ConstraintManager::clearConstraints()
{
    constraints_.clear();
    joint_limit_flag_ = false;
    joint_velocity_limit_flag_ = false;
    relative_pose_flag_ = false;
    tilt_limit_flag_ = false;
    joint_acceleration_limit_flag_ = false;
}

void ConstraintManager::showConstraintList()
{
    std::cout << "Constraint List:" << std::endl;
    for (auto &constraint : constraints_)
    {
        std::cout << "Constraint type: " << constraintType2String(constraint.type)
                  << ", Priority: " << constraint.priority << std::endl;
    }
}

void ConstraintManager::loadControlParams(const std::string control_params_config)
{
    if (!std::ifstream(control_params_config))
    {
        std::cerr << "Control parameters config file not found: " << control_params_config
                  << ". Set parameters to default value " << std::endl;
        return;
    }
    YAML::Node config = YAML::LoadFile(control_params_config);

    // Load gains with default values
    try { K_joint_tracking_ = config["K_joint_tracking"].as<double>(); }
    catch (...) { std::cout << "K_joint_tracking set to default: " << K_joint_tracking_ << std::endl; }

    try { K_joint_limit_ = config["K_joint_limit"].as<double>(); }
    catch (...) { std::cout << "K_joint_limit set to default: " << K_joint_limit_ << std::endl; }

    try { K_collision_avoidance_ = config["K_collision_avoidance"].as<double>(); }
    catch (...) { std::cout << "K_collision_avoidance set to default: " << K_collision_avoidance_ << std::endl; }

    try { K_singularity_ = config["K_singularity"].as<double>(); }
    catch (...) { std::cout << "K_singularity set to default: " << K_singularity_ << std::endl; }

    // Load joint limits
    try {
        std::vector<double> q_lb_vec = config["q_lb"].as<std::vector<double>>();
        std::vector<double> q_ub_vec = config["q_ub"].as<std::vector<double>>();
        if ((q_lb_vec.size() != 14) || (q_ub_vec.size() != 14)) {
            std::cerr << "ERROR: joint limit size does not match with joint size." << std::endl;
            return;
        }
        for (int i = 0; i < 14; i++) {
            q_lb_(i) = q_lb_vec[i];
            q_ub_(i) = q_ub_vec[i];
        }
        joint_limit_flag_ = true;
    } catch (...) {}

    try {
        std::vector<double> dq_lb_vec = config["dq_lb"].as<std::vector<double>>();
        std::vector<double> dq_ub_vec = config["dq_ub"].as<std::vector<double>>();
        if ((dq_lb_vec.size() != 14) || (dq_ub_vec.size() != 14)) {
            std::cerr << "ERROR: joint velocity limit size does not match with joint size." << std::endl;
            return;
        }
        for (int i = 0; i < 14; i++) {
            dq_lb_(i) = dq_lb_vec[i];
            dq_ub_(i) = dq_ub_vec[i];
        }
        joint_velocity_limit_flag_ = true;
    } catch (...) {}

    try {
        std::vector<double> ddq_lb_vec = config["ddq_lb"].as<std::vector<double>>();
        std::vector<double> ddq_ub_vec = config["ddq_ub"].as<std::vector<double>>();
        if ((ddq_lb_vec.size() != 14) || (ddq_ub_vec.size() != 14)) {
            std::cerr << "ERROR: joint acceleration limit size does not match with joint size." << std::endl;
            return;
        }
        for (int i = 0; i < 14; i++) {
            ddq_lb_(i) = ddq_lb_vec[i];
            ddq_ub_(i) = ddq_ub_vec[i];
        }
        joint_acceleration_limit_flag_ = true;
    } catch (...) {}
}

void ConstraintManager::setJointLimit(const Vector14d &q_lb, const Vector14d &q_ub)
{
    q_lb_ = q_lb;
    q_ub_ = q_ub;
    joint_limit_flag_ = true;
    std::cout << "ConstraintManager: Joint limit set." << std::endl;
}

void ConstraintManager::setJointVelocityLimit(const Vector14d &dq_lb, const Vector14d &dq_ub)
{
    dq_lb_ = dq_lb;
    dq_ub_ = dq_ub;
    joint_velocity_limit_flag_ = true;
    std::cout << "ConstraintManager: Joint velocity limit set." << std::endl;
}

void ConstraintManager::setJointAccelerationLimit(const Vector14d &ddq_lb, const Vector14d &ddq_ub)
{
    ddq_lb_ = ddq_lb;
    ddq_ub_ = ddq_ub;
    std::cout << "ConstraintManager: Joint acceleration limit set." << std::endl;
}

void ConstraintManager::setRelativePose(const Matrix4d &T)
{
    T_rel_ = T;
    R_rel_ = T.block<3, 3>(0, 0);
    p_rel_ = T.block<3, 1>(0, 3);
    relative_pose_flag_ = true;
    std::cout << "ConstraintManager: Relative Pose set to: " << T << std::endl;
}

void ConstraintManager::setTiltLimit(const Matrix3d &R1_grasp, const Matrix3d &R2_grasp, double max_tilt)
{
    R1_grasp_ = R1_grasp;
    R2_grasp_ = R2_grasp;
    max_tilt_ = max_tilt;
    tilt_limit_flag_ = true;
    std::cout << "ConstraintManager: Robot1 Grasping Rotation set to: " << R1_grasp_ << std::endl;
    std::cout << "ConstraintManager: Robot2 Grasping Rotation set to: " << R2_grasp_ << std::endl;
    std::cout << "ConstraintManager: Tilt limit set to: " << max_tilt_ << std::endl;
}
// ─────────────────────────────────────────────────────────────────────────────
// Generate Constraints for HQP
// ─────────────────────────────────────────────────────────────────────────────
void ConstraintManager::generateConstraints2HQP(const Vector14d &q, const Vector14d &dq_last,
                                                std::vector<HQP::PriorityConstraint> &priority_constraints)
{
    priority_constraints.clear();
    if (constraints_.size() == 0) {
        std::cerr << "ERROR: No constraints in ConstraintManager" << std::endl;
        return;
    }

    // Find maximum priority
    int max_priority = 0;
    for (const auto &constraint : constraints_) {
        if (constraint.priority > max_priority) {
            max_priority = constraint.priority;
        }
    }

    // Group constraints by priority
    std::vector<std::vector<int>> priority_list(max_priority + 1);
    for (int i = 0; i < constraints_.size(); i++) {
        priority_list[constraints_[i].priority].push_back(i);
    }

    // Generate constraint matrices at each priority level
    for (int p = 0; p < priority_list.size(); p++) {
        if (priority_list[p].size() == 0) {
            if (p > 0) {
                priority_list.erase(priority_list.begin() + p);
                --p;
            }
            continue;
        }

        MatrixXd Cp;
        VectorXd lbp, ubp;
        for (int i = 0; i < priority_list[p].size(); i++) {
            MatrixXd Cpi;
            VectorXd lbpi, ubpi;

            switch (constraints_[priority_list[p][i]].type) {
            case ConstraintType::JOINT_LIMIT_WITH_ACC:
                if (!(joint_limit_flag_ && joint_velocity_limit_flag_ && joint_acceleration_limit_flag_)) {
                    std::cout << "WARNING: Joint limits not set. Ignore constraint." << std::endl;
                    break;
                }
                joint_limit_constraint_with_acc(q, dq_last, Cpi, lbpi, ubpi);
                break;

            case ConstraintType::COLLISION_AVOIDANCE_ALLCBF:
                if (!collision_env_.isEmpty()) {
                    auto start_time = std::chrono::high_resolution_clock::now();
                    collision_avoidance_constraint_ALLCBF(q, Cpi, lbpi, ubpi);
                    auto end_time = std::chrono::high_resolution_clock::now();
                    collision_computation_time_ = std::chrono::duration<double>(end_time - start_time).count();
                }
                break;

            case ConstraintType::SINGULARITY_AVOIDANCE:
                // TODO: Implement singularity avoidance after adding manipulability methods to PandaRobot
                std::cout << "WARNING: Singularity avoidance not yet implemented." << std::endl;
                break;

            default:
                std::cout << "Constraint type not yet implemented: "
                          << constraintType2String(constraints_[priority_list[p][i]].type) << std::endl;
                break;
            }

            // Stack constraints
            if (Cpi.rows() > 0) {
                if (Cp.rows() == 0) {
                    Cp = Cpi;
                    lbp = lbpi;
                    ubp = ubpi;
                } else {
                    Cp.conservativeResize(Cp.rows() + Cpi.rows(), Cp.cols());
                    lbp.conservativeResize(lbp.size() + lbpi.size());
                    ubp.conservativeResize(ubp.size() + ubpi.size());

                    Cp.bottomRows(Cpi.rows()) = Cpi;
                    lbp.tail(lbpi.size()) = lbpi;
                    ubp.tail(ubpi.size()) = ubpi;
                }
            }
        }

        if (Cp.rows() > 0) {
            HQP::PriorityConstraint pc(p, Cp, lbp, ubp);
            priority_constraints.push_back(pc);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Generate Cost Function for QP
// ─────────────────────────────────────────────────────────────────────────────
void ConstraintManager::generateCost(const Vector14d &q, const Vector14d &q_r, const Vector14d &dq_r,
                                      MatrixXd &H, VectorXd &f)
{
    H = MatrixXd::Identity(14, 14) * K_joint_tracking_;
    f = -K_joint_tracking_ * (q_r - q);
}

// ─────────────────────────────────────────────────────────────────────────────
// Joint Limit Constraint with Acceleration
// ─────────────────────────────────────────────────────────────────────────────
void ConstraintManager::joint_limit_constraint_with_acc(const Vector14d &q, const Vector14d &dq_last,
                                                         MatrixXd &C, VectorXd &lb, VectorXd &ub)
{
    C = MatrixXd::Identity(14, 14);
    lb.resize(14);
    ub.resize(14);

    for (int i = 0; i < 14; i++) {
        double lower_bound = dq_lb_(i);
        double upper_bound = dq_ub_(i);

        // Joint position limit: K * (q_lb - q) <= dq <= K * (q_ub - q)
        double pos_lower = K_joint_limit_ * (q_lb_(i) - q(i));
        double pos_upper = K_joint_limit_ * (q_ub_(i) - q(i));

        lower_bound = std::max(lower_bound, pos_lower);
        upper_bound = std::min(upper_bound, pos_upper);

        // Acceleration limit: ddq_lb*dt + dq_last <= dq <= ddq_ub*dt + dq_last
        double acc_lower = ddq_lb_(i) * control_period_ + dq_last(i);
        double acc_upper = ddq_ub_(i) * control_period_ + dq_last(i);

        lower_bound = std::max(lower_bound, acc_lower);
        upper_bound = std::min(upper_bound, acc_upper);

        lb(i) = lower_bound;
        ub(i) = upper_bound;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Collision Avoidance Constraint (ALLCBF)
// ─────────────────────────────────────────────────────────────────────────────
void ConstraintManager::collision_avoidance_constraint_ALLCBF(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub)
{
    MatrixXd grad1, grad2;
    VectorXd d1, d2;

    // Get pairwise distance gradients
    collision_env_.robot2EnvDistanceGradient_Pairwise(robot1_, q.segment(0, 7), d1, grad1);
    collision_env_.robot2EnvDistanceGradient_Pairwise(robot2_, q.segment(7, 7), d2, grad2);

    // Create masks for active constraints
    VectorXi mask1 = (d1.array() < d_active_).cast<int>();
    VectorXi mask2 = (d2.array() < d_active_).cast<int>();

    int N1_active = mask1.sum();
    int N2_active = mask2.sum();

    C.resize(N1_active + N2_active, 14);
    lb.resize(N1_active + N2_active);
    ub.resize(N1_active + N2_active);
    C.setZero();
    lb.setConstant(-std::numeric_limits<double>::infinity());
    ub.setZero();

    // Add active constraints for robot1
    int row_idx = 0;
    for (int i = 0; i < d1.size(); ++i) {
        if (mask1(i)) {
            C.row(row_idx).segment(0, 7) = -grad1.row(i);
            ub(row_idx) = K_collision_avoidance_ * (d1(i) - d_safe_);
            row_idx++;
        }
    }

    // Add active constraints for robot2
    for (int i = 0; i < d2.size(); ++i) {
        if (mask2(i)) {
            C.row(row_idx).segment(7, 7) = -grad2.row(i);
            ub(row_idx) = K_collision_avoidance_ * (d2(i) - d_safe_);
            row_idx++;
        }
    }

    // Update distance info
    d_robot1_ = d1.minCoeff();
    d_robot2_ = d2.minCoeff();
    safe_index1_ = d_robot1_ - d_safe_;
    safe_index2_ = d_robot2_ - d_safe_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Singularity Avoidance Constraint
// ─────────────────────────────────────────────────────────────────────────────
void ConstraintManager::singularity_avoidance_constraint(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub)
{
    // TODO: Implement after adding manipulabilityIndex and manipulabilityGradient methods to PandaRobot
    // For now, create empty constraints
    C.resize(0, 14);
    lb.resize(0);
    ub.resize(0);

    std::cout << "WARNING: Singularity avoidance constraint called but not yet implemented." << std::endl;
}
