#include "franka_example_controllers/utils/hierarchical_qp.h"
#include <iostream>

namespace HQP
{

    HierarchicalQP::HierarchicalQP()
    {
        osqp_solver_.settings()->setVerbosity(false);
        osqp_solver_.settings()->setMaxIteration(5000);
        /* large tolerance will cause the subsequent QP problem infeasible */
        osqp_solver_.settings()->setAbsoluteTolerance(1e-4);
        osqp_solver_.settings()->setRelativeTolerance(1e-4);
        osqp_solver_.settings()->setPolish(true); // this helps refine the solution to avoid infeasibility
        osqp_solver_.settings()->setScaling(true);
        osqp_solver_.settings()->setWarmStart(true);
    }

    void HierarchicalQP::setCost(const MatrixXd &H, const VectorXd &f)
    {
        if (H.rows() != H.cols())
        {
            std::cerr << "HQP Error: H.rows() != H.cols()." << std::endl;
            return;
        }
        if (H.cols() != f.size())
        {
            std::cerr << "HQP Error: H.rows() != f.rows()" << std::endl;
            return;
        }
        if (H.cols() <= 0)
        {
            std::cerr << "HQP Error: nx_ <= 0" << std::endl;
            return;
        }
        H_ = H;
        f_ = f;
        nx_ = H.cols();
    }

    void HierarchicalQP::addConstraint(PriorityConstraint &constraint)
    {
        if (constraint.C.rows() == 0)
        {
            std::cerr << "HQP Error: Empty Constraint!" << std::endl;
            return;
        }
        if ((constraint.C.rows() != constraint.lb.rows()) || (constraint.C.rows() != constraint.ub.rows()))
        {
            std::cerr << "HQP Error: Inconsistent dimension in constraint" << std::endl;
            return;
        }
        if (constraint.C.cols() != nx_)
        {
            std::cerr << "HQP Error: C.cols() != nx_." << std::endl;
            return;
        }
        const double eps = 1e-9;
        if ((constraint.ub.array() + eps <= constraint.lb.array()).any())
        {
            std::cerr << "HQP Error: ub < lb." << std::endl;
            std::cout << "ub: " << constraint.ub.transpose() << std::endl;
            std::cout << "lb: " << constraint.lb.transpose() << std::endl;
            return;
        }
        constraints_.push_back(constraint);
        n_constraints_ += constraint.C.rows();
    }

    void HierarchicalQP::setConstraints(std::vector<PriorityConstraint> &constraints)
    {
        /* This will overwrite the constraints_ vector */
        n_constraints_ = 0;
        for (auto &constraint : constraints)
        {
            if (constraint.C.rows() == 0)
            {
                std::cerr << "HQP Error: Empty Constraint!" << std::endl;
                return;
            }
            if ((constraint.C.rows() != constraint.lb.rows()) || (constraint.C.rows() != constraint.ub.rows()))
            {
                std::cerr << "HQP Error: Inconsistent dimension in constraint" << std::endl;
                return;
            }
            if (constraint.C.cols() != nx_)
            {
                std::cerr << "HQP Error: C.cols() != nx_." << std::endl;
                return;
            }
            const double eps = 1e-9;
            if ((constraint.ub.array() + eps <= constraint.lb.array()).any())
            {
                std::cerr << "HQP Error: ub < lb." << std::endl;
                std::cout << "ub: " << constraint.ub.transpose() << std::endl;
                std::cout << "lb: " << constraint.lb.transpose() << std::endl;
                return;
            }
            n_constraints_ += constraint.C.rows();
        }
        constraints_ = constraints;
        // std::cout << "Set constraints successfully." << std::endl;
    }

    void HierarchicalQP::solve(HQPSolverResult &result)
    {
        std::sort(constraints_.begin(), constraints_.end(), [](const PriorityConstraint &a, const PriorityConstraint &b)
                  { return a.priority < b.priority; }); // sort constraints according to priority
        int N_priority = constraints_.size();
        if (N_priority == 0)
        {
            std::cerr << "HQP Error: No constraints added." << std::endl;
            result.success = false;
            return;
        }
        MatrixXd C_tilde;
        VectorXd lb_tilde, ub_tilde;
        if (constraints_[0].priority == 0) // Hard constraints
        {
            C_tilde = constraints_[0].C;
            lb_tilde = constraints_[0].lb;
            ub_tilde = constraints_[0].ub;
        }
        VectorXd x = VectorXd::Zero(nx_);
        int nx = nx_;
        int nw; // size of slack variable for inequality constraints
        for (int p = 0; p < N_priority; p++)
        {
            /********** Construct QP subproblem **********/
            /*  z = [x; w];
                min 1/2 z^T P z + q^T z
                s.t. lb_z <= C_z z <= ub_z

                where
                P = [rho , 0;
                     0,    I]
                q = 0;
                C_z = [C_tilde, 0;
                       C_p,    I]
                lb_z = [lb_tilde;
                        lb_p]
                ub_z = [ub_tilde;
                        ub_p]
            */
            if (constraints_[p].priority == 0)
            {
                continue; // Potential Bug: 如果有多个hard constraint，只会添加第一个，后面的都会被忽略
            }
            // std::cout << "HQP Iteration: Priority " << constraints_[p].priority << std::endl;
            nw = constraints_[p].C.rows();
            VectorXd z(nx + nw);
            z << x, VectorXd::Zero(nw);

            // Initialize slack variable, w = max(0, lb - C*x) + min(0, C*x - ub)
            // VectorXd w_lb,w_ub;
            // w_lb = (-constraints_[p].C * x + constraints_[p].lb).array().max(0).matrix();
            // w_ub = (constraints_[p].C * x - constraints_[p].ub).array().min(0).matrix();
            // z << x, w_lb + w_ub;

            // Construct P,q
            MatrixXd P = MatrixXd::Zero(nx + nw, nx + nw);
            VectorXd q = VectorXd::Zero(nx + nw);

            P.block(0, 0, nx, nx) = rho_ * MatrixXd::Identity(nx, nx);
            P.block(nx, nx, nw, nw) = MatrixXd::Identity(nw, nw);

            // Construct C_z, lb_z, ub_z

            MatrixXd C_z = MatrixXd::Zero(C_tilde.rows() + nw, nx + nw);
            C_z.block(0, 0, C_tilde.rows(), C_tilde.cols()) = C_tilde;
            C_z.block(C_tilde.rows(), 0, nw, nx) = constraints_[p].C;
            C_z.block(C_tilde.rows(), nx, nw, nw) = MatrixXd::Identity(nw, nw);

            VectorXd lb_z(lb_tilde.size() + nw), ub_z(ub_tilde.size() + nw);
            lb_z << lb_tilde, constraints_[p].lb;
            ub_z << ub_tilde, constraints_[p].ub;

            /********** Solve QP subproblem **********/
            // std::cout << "Solve the QP sub-problem." << std::endl;
            bool success = solveQP(P, q, C_z, lb_z, ub_z, z);
            if (!success)
            {
                result.success = false;
                return;
            }
            x = z.head(nx);

            /********** Check constraint violation **********/
            VectorXd w = z.tail(nw);
            result.w.conservativeResize(result.w.size() + nw);
            result.w.tail(nw) = w;

            double residual = w.norm();
            if (residual > constraint_violation_tol_)
            {
                std::cout << "HQP: Constraints cannot be fully satisfied. Residual: " << residual
                          << " Priority: " << constraints_[p].priority << std::endl;
                result.constraint_violated = true;
            }

            /********** check infeasibility **********/
            double primal_inf = std::max((lb_tilde - C_tilde * x).array().maxCoeff(), (C_tilde * x - ub_tilde).array().maxCoeff());
            if (primal_inf >= primal_infeasible_tol_)
            {
                std::cout << "HQP Warning! QP solution violates primal infeasibility: Residual: " << primal_inf << std::endl;
            }
            // relax the constraints to guarantee feasibility
            // Potential Problem: 这里会导致约束被放宽
            lb_tilde = lb_tilde.array().min((C_tilde * x).array()).matrix();
            ub_tilde = ub_tilde.array().max((C_tilde * x).array()).matrix();
            /********** Update subspace **********/
            int n = C_tilde.rows();
            C_tilde.conservativeResize(n + constraints_[p].C.rows(), nx);
            C_tilde.block(n, 0, constraints_[p].C.rows(), nx) = constraints_[p].C;
            lb_tilde.conservativeResize(n + constraints_[p].lb.size());
            ub_tilde.conservativeResize(n + constraints_[p].ub.size());

            for (int i = 0; i < constraints_[p].C.rows(); i++)
            {
                double Cx = constraints_[p].C.row(i).dot(x);
                // Potential Bug: 这种做法只是平移了约束，没有真正的"松弛"，但是这种平移对等式和单边不等式约束的效果与"松弛"是一样的
                /*
                    // lb_tilde(n + i) = constraints_[p].lb(i) - w(i);
                    // ub_tilde(n + i) = constraints_[p].ub(i) - w(i);
                    注意这里不能直接按照 lb-w, ub-w 来更新lb_tilde和ub_tilde，因为有数值误差
                */
                if (Cx < constraints_[p].lb(i))
                {
                    lb_tilde(n + i) = Cx;
                    ub_tilde(n + i) = constraints_[p].ub(i) + Cx - constraints_[p].lb(i);
                }
                else if (Cx > constraints_[p].ub(i))
                {
                    lb_tilde(n + i) = constraints_[p].lb(i) + Cx - constraints_[p].ub(i);
                    ub_tilde(n + i) = Cx;
                }
                else
                {
                    lb_tilde(n + i) = constraints_[p].lb(i);
                    ub_tilde(n + i) = constraints_[p].ub(i);
                }
            }
        }
        // solve the final QP problem
        // std::cout << "Solve the final QP Problem." << std::endl;

        bool success = solveQP(H_, f_, C_tilde, lb_tilde, ub_tilde, x);
        result.success = success;
        result.x = x;
        return;
    }
    /*
        The QP subproblem is solved by OSQP, the problem fomulation is:
        min 1/2 x^T P x + q^T x
        s.t. lb <= Cx <= ub
    */
    bool HierarchicalQP::solveQP(const MatrixXd &P, const VectorXd &q,
                                 const MatrixXd &C, const VectorXd &lb, const VectorXd &ub,
                                 VectorXd &x)
    {

        Eigen::SparseMatrix<double> P_sparse = P.sparseView();
        Eigen::SparseMatrix<double> C_sparse = C.sparseView();
        Eigen::VectorXd gradient = q;
        Eigen::VectorXd l = lb;
        Eigen::VectorXd u = ub;

        osqp_solver_.clearSolver();
        osqp_solver_.data()->clearHessianMatrix();  // 必须保留！
        osqp_solver_.data()->clearLinearConstraintsMatrix();

        osqp_solver_.data()->setNumberOfVariables(x.size());
        osqp_solver_.data()->setNumberOfConstraints(C.rows());
        osqp_solver_.data()->setHessianMatrix(P_sparse);
        osqp_solver_.data()->setGradient(gradient);
        osqp_solver_.data()->setLinearConstraintsMatrix(C_sparse);
        osqp_solver_.data()->setLowerBound(l);
        osqp_solver_.data()->setUpperBound(u);

        if (!osqp_solver_.initSolver())
        {
            std::cout << "Error while initializing solver" << std::endl;
        }
        osqp_solver_.setPrimalVariable(x); // warm start

        if (osqp_solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            std::cout << "Error while solving QP problem" << std::endl;
            return false;
        }
        OsqpEigen::Status status = osqp_solver_.getStatus();

        // TODO SolvedInaccurate和MaxIterReached也认为是求解成功！
        if ((status == OsqpEigen::Status::Solved) || (status == OsqpEigen::Status::SolvedInaccurate) || (status == OsqpEigen::Status::MaxIterReached))
        {
            x = osqp_solver_.getSolution();
            switch (status)
            {
            case (OsqpEigen::Status::MaxIterReached):
            {
                std::cout << "OSQP WARNING: Maximum iteration reached." << std::endl;
                break;
            }
            case (OsqpEigen::Status::SolvedInaccurate):
            {
                std::cout << "OSQP WARNING: Solved inaccurate." << std::endl;
                break;
            }
            }
            return true;
        }
        else
        {
            switch (status)
            {
            case (OsqpEigen::Status::DualInfeasibleInaccurate):
            {
                std::cout << "OSQP ERROR: Dual Infeasible Inaccurate." << std::endl;
                break;
            }
            case (OsqpEigen::Status::PrimalInfeasibleInaccurate):
            {
                std::cout << "OSQP ERROR: Primal Infeasible Inaccurate." << std::endl;
                break;
            }
            case (OsqpEigen::Status::PrimalInfeasible):
            {
                std::cout << "OSQP ERROR: Primal Infeasible." << std::endl;
                break;
            }
            case (OsqpEigen::Status::DualInfeasible):
            {
                std::cout << "OSQP ERROR: Dual Infeasible." << std::endl;
                break;
            }
            case (OsqpEigen::Status::Unsolved):
            {
                std::cout << "OSQP ERROR: Unsolved." << std::endl;
                break;
            }
            case (OsqpEigen::Status::NonCvx):
            {
                std::cout << "OSQP ERROR: Non Convex." << std::endl;
                break;
            }
            default:  // Other errors
            {
                std::cout << "OSQP ERROR: Unknown Error!" << std::endl;
                break;
            }
            }
            Eigen::VectorXd x_sol = osqp_solver_.getSolution();
            // for debug
            std::cout << "x_init: " << x << std::endl;
            std::cout << "P: " << P << std::endl;
            std::cout << "q: " << q << std::endl;
            std::cout << "C: " << C << std::endl;
            std::cout << "l: " << lb << std::endl;
            std::cout << "u: " << ub << std::endl;
            std::cout << "x_sol: " << x_sol << std::endl;

            return false;
        }
    }

    void HierarchicalQP::clear()
    {
        constraints_.clear();
    }

} // namespace HQP
