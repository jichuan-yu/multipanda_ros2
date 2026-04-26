/*
    Author: Jichuan Yu
    Date: 2024.10
    Adapted for franka_example_controllers package
*/
#ifndef FRANKA_EXAMPLE_CONTROLLERS_CONSTRAINT_MANAGER_H
#define FRANKA_EXAMPLE_CONTROLLERS_CONSTRAINT_MANAGER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "franka_example_controllers/utils/hierarchical_qp.h"
#include "franka_example_controllers/utils/robot_kinematics.hpp"
#include "franka_example_controllers/utils/collision_env.h"

using namespace Eigen;

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector14d = Eigen::Matrix<double, 14, 1>;

enum class ConstraintType
{
    JOINT_LIMIT,                // without acceleration limit
    JOINT_LIMIT_WITH_ACC,       // with acceleration limit
    RELATIVE_POSE,              // Equality constraint is better!
    RELATIVE_POSE_CLF,          // Construct constraint by Control Lyapunov Function. Do not use!!! unstable!!!!
    TILT_LIMIT,                 // Tilt limit constraint (object z-axis w.r.t world z-axis)
    COLLISION_AVOIDANCE_CBF,    // Whole-body collision avoidance using a single CBF
    COLLISION_AVOIDANCE_ALLCBF, // Construct CBF for each collision pair, This can be very slow!
    COLLISION_AVOIDANCE_TVCBF,  // Consider Velocity of the dynamic obstacle
    MUTUAL_COLLISION_AVOIDANCE,
    SINGULARITY_AVOIDANCE, // Avoid singularity of the robot
};

std::string constraintType2String(const ConstraintType &type);

struct Constraint
{
    int priority;
    ConstraintType type;
    // constructor
    Constraint(int priority, ConstraintType type)
        : priority(priority), type(type) {}
};

/**************    IMPORTANT!!!!!!!!!  **************
    In the constraint list, each type of constraint is only allowed to appear once.
*****************************************************/

class ConstraintManager
{
public:
    ConstraintManager(PandaRobot &robot1, PandaRobot &robot2, CollisionEnv &collision_env,
                      std::string control_params_config);
    ~ConstraintManager();
    void addConstraint(const Constraint &constraint);
    /* Add a constraint and increment the priority of all constraints with a priority >= new constraint's priority by 1.*/
    void insertConstraint(const Constraint &constraint); // 逻辑不对！不要使用！
    void removeConstraint(const ConstraintType &type);
    void clearConstraints();
    void showConstraintList();
    void loadControlParams(const std::string control_params_config);
    void checkConstraintParams(const ConstraintType &type); // output a warning if the corresponding constraint parameters are not set
    /*****************************************************
                set constraint related variables
    *****************************************************/
    void setJointLimit(const Vector14d &q_lb, const Vector14d &q_ub);
    void setJointVelocityLimit(const Vector14d &dq_lb, const Vector14d &dq_ub);
    void setJointAccelerationLimit(const Vector14d &ddq_lb, const Vector14d &ddq_ub);
    void setRelativePose(const Matrix4d &T);
    void setTiltLimit(const Matrix3d &R1_grasp, const Matrix3d &R2_grasp, double max_tilt);

    /*****************************************************
        Generate prioritized constraint matrices
           according to the constraint list
    *****************************************************/
    void generateConstraints2HQP(const Vector14d &q, const Vector14d &dq_last,
                                 std::vector<HQP::PriorityConstraint> &priority_constraints);
    /*****************************************************
        Generate prioritized constraint matrices for HardSoftQP
           0: hard constraint, 1: soft constraint
        Any priority >=1 will be merged into the soft constraint
    *****************************************************/
    void generateConstraints2HardSoftQP(const Vector14d &q, const Vector14d &dq_last,
                                        std::vector<HQP::PriorityConstraint> &priority_constraints);
    /*****************************************************
        Joint Limit Constraint
        max{dq_lb,K(q_lb-q)} <= dq <= min{dq_ub,K(q_ub-q)}
        dim = 14
    *****************************************************/
    void joint_limit_constraint(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);
    /*****************************************************
        Joint Limit Constraint with acceleration limit
        max{dq_lb, K(q_lb-q), ddq_lb*dt + dq(t-dt)} <= dq <= min{dq_ub, K(q_ub-q), ddq_ub*dt + dq(t-dt)}
        dim = 14
    *****************************************************/
    void joint_limit_constraint_with_acc(const Vector14d &q, const Vector14d &dq_last, MatrixXd &C, VectorXd &lb, VectorXd &ub);

    /*****************************************************
        Relative Pose Constraint (Equality constraint)
        original form:
        e_rp = 0; // relative position error
        e_ro = 0; // relative orientation error
                 =>
        J_{e_rp} dq = -K_p * e_rp;
        J_{e_ro} dq = -K_o * e_ro;
        dim = 6
    *****************************************************/
    void relative_pose_constraint(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);

    /*****************************************************
        Relative Pose Constraint (Ineq constraint)
        Control Lyapunov Function:
        V_rpi = 1/2 e_rpi^2 = 0; // relative position error

        V_roi = 1/2 e_roi^2 = 0; // relative orientation error
                 =>
        dV_rpi = e_rpi * J_{e_rpi} * dq <= - K_p * V_rpi
        dV_roi = e_roi * J_{e_roi} * dq <= - K_o * V_roi

        dim = 6
    *****************************************************/
    void relative_pose_contraint_CLF(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);

    /*****************************************************
        Relative Pose Error
        e_rp = R1^T*(p2 - p1) - p_rel;
        e_ro = q_ro.vec();
    *****************************************************/
    void relative_pose_error(const Vector14d &q, Vector3d &e_rp, Vector3d &e_ro);
    /* compute the relative pose error w.r.t a desired T_rel*/
    void relative_pose_error(const Vector14d &q, const Matrix4d T_rel_d, Vector3d &e_rp, Vector3d &e_ro);

    /*****************************************************
        Tilt Limit Constraint (based on CBF)
        cos(tilt) = ln^T * z
        h = ln^T * z - cos(max_tilt) >= 0
                 =>
        - dh/dq dq <= K_tilt * h;
        dim = 1
    *****************************************************/
    void tilt_limit_constraint(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);
    /* compute current tilt angle (robot 1)*/
    double tilt_angle1(const Vector14d &q);
    /* compute current tilt angle (robot 2)*/
    double tilt_angle2(const Vector14d &q);
    /********************************
        Collision Avoidance Constraint
        for each robot:
            d(q) - d_safe >= 0
            -Grad dq <= K * (d(q) - d_safe)
        dim = 2 (Do not consider mutual collision at this stage)
     *****************************************************/
    void collision_avoidance_constraint_CBF(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);
    /********************************
        Collision Avoidance Constraint
        for each **collision pair**:
            d(q) - d_safe >= 0
            -Grad dq <= K * (d(q) - d_safe)
        dim = N_collision_pair (Do not consider mutual collision at this stage)
    ****************************************************/
    void collision_avoidance_constraint_ALLCBF(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);
    /********************************
        Collision Avoidance Constraint with Time-Varying CBF
        for each robot:
            d(q,t) - d_safe >= 0
            -Grad dq <= K * (d(q) - d_safe) + Grad_t
        dim = 2 (Do not consider mutual collision at this stage)
     ****************************************************/
    void collision_avoidance_constraint_TVCBF(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);
    /*****************************************************
        Singularity Avoidance Constraint
        for each robot:
        manipIndex(q) - singularity_threshold >= 0
        -Grad dq <= K * (manipIndex(q) - singular_threshold)
        dim = 2
     ****************************************************/
    void singularity_avoidance_constraint(const Vector14d &q, MatrixXd &C, VectorXd &lb, VectorXd &ub);

    double getRobot1CollisionDistance() { return d_robot1_; }
    double getRobot2CollisionDistance() { return d_robot2_; }
    double getRobot1SafeIndex() { return safe_index1_; }
    double getRobot2SafeIndex() { return safe_index2_; }

    // Get collision constraint computation time
    double getCollisionComputationTime() { return collision_computation_time_; }

    /* Generate QP Cost Function */
    void generateCost(const Vector14d &q, const Vector14d &q_r, const Vector14d &dq_r,
                      MatrixXd &H, VectorXd &f);

private:
    std::vector<Constraint> constraints_;
    PandaRobot &robot1_, &robot2_; // Use references to ensure consistency with the Robots in DualArmSafeControllerSim.
    CollisionEnv &collision_env_;
    /*****************************************************
                constraint related variables
    *****************************************************/
    // joint limit and joint velocity limit
    Vector14d q_lb_, q_ub_, dq_lb_, dq_ub_, ddq_lb_, ddq_ub_;
    // relative pose
    Matrix4d T_rel_;
    Vector3d p_rel_;
    Matrix3d R_rel_;
    // object tilt (object z-axis w.r.t world z-axis)
    double max_tilt_;              // rad
    Matrix3d R1_grasp_, R2_grasp_; // grasping rotation matrix (end-effector w.r.t object)
    // safe distance margin
    double d_safe_ = 0.1;   // safe distance margin
    double d_active_ = 0.3; // threshold to activate the constraint (default: 0.3)
    // singularity avoidance
    double singularity_threshold_ = 0.01; // threshold for singularity avoidance

    // distance to the nearest obstacle, for data record
    double d_robot1_ = std::numeric_limits<double>::max();
    double d_robot2_ = std::numeric_limits<double>::max();
    // safe CBF value, for data record
    double safe_index1_ = std::numeric_limits<double>::max();
    double safe_index2_ = std::numeric_limits<double>::max();

    // constraint computation time, for data record (in seconds)
    // Only one collision avoidance constraint can be active at a time
    double collision_computation_time_ = 0.0;

    // gains
    double control_period_ = 0.01; // control period
    double K_joint_tracking_ = 5;  //  joint tracking gain (for cost function)
    double K_joint_limit_ = 100;   //  K/dt (braking time horizon/sampling time)
    double K_collision_avoidance_ = 10;
    double K_relative_position_ = 10;
    double K_relative_orientation_ = 10;
    double K_tilt_limit_ = 10;  // penalty weight for object tilt
    double K_singularity_ = 10; // penalty weight for singularity avoidance

    // penalty weight for soft constraint
    double w_joint_limit_ = 100;         // rad
    double w_collision_avoidance_ = 10;  // m
    double w_relative_position_ = 50;    // m
    double w_relative_orientation_ = 50; // rad
    double w_tilt_limit_ = 50;           // cos(tilt)-cos(max_tilt)
    double w_singularity_ = 50;          // manipIndex - singularity_threshold

    // flags that indicate whether the constraint varaibles are set
    bool joint_limit_flag_ = false;
    bool joint_velocity_limit_flag_ = false;
    bool joint_acceleration_limit_flag_ = false;
    bool relative_pose_flag_ = false;
    bool tilt_limit_flag_ = false;
};

#endif // FRANKA_EXAMPLE_CONTROLLERS_CONSTRAINT_MANAGER_H