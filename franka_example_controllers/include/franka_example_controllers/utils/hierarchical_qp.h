/*
    Author: Jichuan Yu
    Date: 2024.12
    Adapted for franka_example_controllers package
*/
#ifndef FRANKA_EXAMPLE_CONTROLLERS_HIERARCHICAL_QP_H
#define FRANKA_EXAMPLE_CONTROLLERS_HIERARCHICAL_QP_H

#include <Eigen/Dense>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>

/*
    A hierarchical QP problem is defined as:
    min 1/2 x^T H x + f^T x
    s.t.
        lb0 <= C0 x <= ub0    -- priority 0
        lb1 <= C1 x <= ub1    -- priority 1
                ...
*/
/*
    The k-th QP subproblem is described as:
    min  1/2 ||w||^2 + 1/2 rho ||x||^2 (regularization term)
    s.t. lb_tilde <= C_tilde x <= ub_tilde
         lbk <= Ck x + w <= ubk
*/

namespace HQP
{
    using namespace Eigen;

    struct HQPSolverResult
    {
        bool success = false;
        bool constraint_violated = false;
        VectorXd x;
        VectorXd w; // slack variables for all constraints with p>=1
    };

    struct PriorityConstraint
    {
        int priority; // priority = 0 -> hard constraint
        MatrixXd C;
        VectorXd lb;
        VectorXd ub;
        VectorXd penalty_weight; // penalty weight for soft constraint, only used in HardSoftQP
        // default constructor
        PriorityConstraint() : priority(1) {}

        // constructor with parameters
        PriorityConstraint(int priority, const MatrixXd &C, const VectorXd &lb, const VectorXd &ub)
            : priority(priority), C(C), lb(lb), ub(ub) {}
    };

    /*   IMPORTANT!!!!
        constraints_ 中每一个元素都会被看作一个单独的优先级
        如果有多个约束在同一个优先级，请在赋值给HQP之前就合并这些约束
    */
    class HierarchicalQP
    {
    private:
        MatrixXd H_;
        VectorXd f_;
        std::vector<PriorityConstraint> constraints_;
        OsqpEigen::Solver osqp_solver_; // OSQP solver instance

        int nx_ = 0;                             // size of optimization variable
        int n_constraints_ = 0;                  // number of constraints
        double rho_ = 5e-5;                      // Regularization term
        double constraint_violation_tol_ = 1e-2; // if >= tol, output info
        double primal_infeasible_tol_ = 1e-4;    // QP求解的infeasibility，
    public:
        HierarchicalQP();

        void setCost(const MatrixXd &H, const VectorXd &f);
        void addConstraint(PriorityConstraint &constraint);
        void setConstraints(std::vector<PriorityConstraint> &constraints);
        void solve(HQPSolverResult &result);
        bool solveQP(const MatrixXd &P, const VectorXd &q,
                     const MatrixXd &C, const VectorXd &lb, const VectorXd &ub,
                     VectorXd &x);
        int numConstraints() const { return n_constraints_; }
        void clear();
    };

} // namespace HQP

#endif // FRANKA_EXAMPLE_CONTROLLERS_HIERARCHICAL_QP_H
