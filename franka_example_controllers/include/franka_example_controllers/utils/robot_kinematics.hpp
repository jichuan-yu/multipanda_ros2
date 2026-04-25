/*
    Author: Jichuan Yu
    Date: 2024.10
    Refactored for franka_example_controllers by GitHub Copilot
*/
#ifndef FRANKA_EXAMPLE_CONTROLLERS_ROBOT_KINEMATICS_H
#define FRANKA_EXAMPLE_CONTROLLERS_ROBOT_KINEMATICS_H

#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <algorithm>

namespace franka_example_controllers {

using namespace Eigen;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using CollisionSphere = std::pair<Vector3d, double>;

class PandaRobot {
public:
  PandaRobot(int id, const std::string& collision_config_path = "");

  int id; // robot id: 1/2
  /* joint limits */
  Vector7d q_lb;
  Vector7d q_ub;

  void setBase(const Vector3d& p_base, const Vector3d& eul_base);
  int njoint() const { return njoint_; };
  int numSpheres() const { return n_collision_spheres_ + n_attached_spheres_; };
  const Vector3d& basePosition() const { return p_base_; };
  const Matrix3d& baseRotation() const { return R_base_; };
  const Matrix4d& baseTransform() const { return T_base_; };
  void loadCollisionSpheres(const std::string& config_path);
  void detachCollisionSpheres() {
    attached_spheres_.clear();
    n_attached_spheres_ = 0;
  }
  void attachCollisionSpheres(std::vector<CollisionSphere>& spheres) {
    attached_spheres_ = spheres;
    n_attached_spheres_ = attached_spheres_.size();
  }

  /******************** Kinematics functions ********************/
  void getR(const Vector7d& q, Matrix3d& R) const;                      // w.r.t. world frame
  void getT(const Vector7d& q, Matrix4d& T) const;                      // w.r.t. world frame
  void getJacobian(const Vector7d& q, MatrixXd& J) const;               // w.r.t. **base** frame, J = [Jp; Jo] 6x7 matrix
  void getJacobian_world(const Vector7d& q, MatrixXd& J) const;         // w.r.t. world frame, J = [Jp; Jo] 3x7 matrix

  /******************** collision detection functions ********************/
  void getCollisionSpheres(const Vector7d& q, std::vector<CollisionSphere>& collision_spheres_current) const;

  /*
      Get the collision spheres and their Jacobians w.r.t. world frame
      J = Jp 3x7 matrix
  */
  void getCollisionSpheres_Jacobians(const Vector7d& q,
                                     std::vector<CollisionSphere>& collision_spheres_current,
                                     std::vector<MatrixXd>& Jacobians) const;

  /*
      Get the Jacobian matrix of the i-th collision sphere w.r.t. world frame
      J = Jp 3x7 matrix
  */
  void getCollisionSphereJacobian(const Vector7d& q, MatrixXd& J, int sphere_index) const;

  /*
      Manipulation Index of end-effector
  */
  double getManipIndex(const Vector7d& q) const;

  /*
      Get Manipulation Index and Gradient
      Using auto-differentiation to compute the gradient
  */
  void getManipIndexAndGrad(const Vector7d& q, double& manipIndex, Vector7d& grad) const;

private:
  int njoint_;
  MatrixXd DH_; // DH parameters

  /* base frame (w.r.t. world frame) */
  Vector3d p_base_;
  Vector3d eul_base_;
  Matrix3d R_base_;
  Matrix4d T_base_;

  /* collision spheres */
  std::string collision_spheres_config_;
  std::vector<std::vector<CollisionSphere>> collision_spheres_;
  std::vector<CollisionSphere> attached_spheres_; // spheres attached to the end-effector frame
  int n_collision_spheres_;
  int n_attached_spheres_;
};

/*
    Compute the relative pose
    T_rel = T1^{-1}*T2;
*/
void relativePose(const PandaRobot& robot1,
                  const PandaRobot& robot2,
                  const Vector7d& q1,
                  const Vector7d& q2,
                  Matrix4d& T_rel);

/*
    Compute the relative Jacobian matrix (6x14 matrix): EE of robot2 w.r.t. EE of robot1
    Jrel = [-Psi*Omega{b1 w.r.t. e1}*J1, Omega{b2 w.r.t. e1}*J2]
*/
void relativeJacobian(const PandaRobot& robot1,
                      const PandaRobot& robot2,
                      const Vector7d& q1,
                      const Vector7d& q2,
                      MatrixXd& Jrel);

Matrix3d skew(const Vector3d& v); // skew-symmetric matrix of a vector

Matrix3d eul2Rotm(const Vector3d& eul); // ‘XYZ' Euler angles to rotation matrix

} // namespace franka_example_controllers

#endif // FRANKA_EXAMPLE_CONTROLLERS_ROBOT_KINEMATICS_H
