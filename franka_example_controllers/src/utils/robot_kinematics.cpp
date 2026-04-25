#include "franka_example_controllers/utils/robot_kinematics.hpp"

namespace franka_example_controllers {

PandaRobot::PandaRobot(int id, const std::string& collision_config_path) : id(id) {
  njoint_ = 7;

  p_base_ = Vector3d::Zero();
  eul_base_ = Vector3d::Zero();
  R_base_ = Matrix3d::Identity();
  T_base_ = Matrix4d::Identity();

  /* Panda DH parameters (Craig's convention) */
  DH_.resize(8, 4);
  DH_ << 0, 0.333, 0, 0,
         0, 0, 0, -M_PI * 0.5,
         0, 0.316, 0, M_PI * 0.5,
         0, 0, 0.0825, M_PI * 0.5,
         0, 0.384, -0.0825, -M_PI * 0.5,
         0, 0, 0, M_PI * 0.5,
         0, 0, 0.088, M_PI * 0.5,
         -0.7854, 0.107, 0, 0; // flange frame, but rotate pi/4 to align with the hand frame
  /*  theta,      d,          a,        alpha*/

  q_lb << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_ub << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

  if (!collision_config_path.empty()) {
    loadCollisionSpheres(collision_config_path);
  }
}

void PandaRobot::loadCollisionSpheres(const std::string& config_path) {
  collision_spheres_config_ = config_path;
  YAML::Node config;
  try {
    config = YAML::LoadFile(collision_spheres_config_);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load collision spheres from " << config_path << ": " << e.what() << std::endl;
    return;
  }

  YAML::Node conf = config["collision_spheres"];
  if (!conf) {
    std::cerr << "No 'collision_spheres' node in " << config_path << std::endl;
    return;
  }

  std::vector<std::string> keys;
  for (YAML::const_iterator it = conf.begin(); it != conf.end(); ++it) {
    keys.push_back(it->first.as<std::string>());
  }
  int n_links = keys.size();
  if (n_links != DH_.rows()) {
    std::cerr << "Number of Links must be identical to DH rows. Failed to load collision spheres" << std::endl;
    return;
  }

  collision_spheres_.clear();
  int n_spheres = 0;
  for (const auto& key : keys) {
    YAML::Node sphere_each_link = conf[key];
    std::vector<CollisionSphere> link_spheres;

    for (std::size_t j = 0; j < sphere_each_link.size(); ++j) {
      YAML::Node sphere = sphere_each_link[j];
      double radius = sphere["radius"].as<double>();
      std::vector<double> center = sphere["center"].as<std::vector<double>>();
      Vector3d center_vec(center[0], center[1], center[2]);

      link_spheres.emplace_back(center_vec, radius);
      n_spheres++;
    }
    collision_spheres_.push_back(link_spheres);
  }
  n_collision_spheres_ = n_spheres;
  std::cout << "Loaded " << n_spheres << " collision spheres for Panda robot." << std::endl;
}

void PandaRobot::setBase(const Vector3d& p_base, const Vector3d& eul_base) {
  p_base_ = p_base;
  eul_base_ = eul_base;
  R_base_ = eul2Rotm(eul_base_);
  T_base_.block<3, 3>(0, 0) = R_base_;
  T_base_.block<3, 1>(0, 3) = p_base_;
}

void PandaRobot::getR(const Vector7d& q, Matrix3d& R) const {
  MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;

  R = R_base_;

  for (int i = 0; i < DH.rows(); i++) {
    Matrix3d Ri;
    Ri << cos(DH(i, 0)), -sin(DH(i, 0)), 0,
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3));
    R *= Ri;
  }
}

void PandaRobot::getT(const Vector7d& q, Matrix4d& T) const {
  MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;

  T = T_base_;

  for (int i = 0; i < DH.rows(); i++) {
    Matrix4d Ti;
    Ti << cos(DH(i, 0)), -sin(DH(i, 0)), 0, DH(i, 2),
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)), -DH(i, 1) * sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1) * cos(DH(i, 3)),
          0, 0, 0, 1;
    T *= Ti;
  }
}

void PandaRobot::getJacobian(const Vector7d& q, MatrixXd& J) const {
  MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;

  J = MatrixXd::Zero(6, njoint_);

  Matrix4d T = Matrix4d::Identity();
  std::vector<Matrix4d> T_all(DH.rows(), Matrix4d::Identity());

  for (int i = 0; i < DH.rows(); ++i) {
    Matrix4d Ti;
    Ti << cos(DH(i, 0)), -sin(DH(i, 0)), 0, DH(i, 2),
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)), -DH(i, 1) * sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1) * cos(DH(i, 3)),
          0, 0, 0, 1;
    T *= Ti;
    T_all[i] = T;
  }
  for (int i = 0; i < njoint_; ++i) {
    Matrix4d T_prev = T_all[i];
    Vector3d z = T_prev.block<3, 1>(0, 2);
    Vector3d pe = T.block<3, 1>(0, 3);
    Vector3d pi = T_prev.block<3, 1>(0, 3);
    Vector3d Jp = z.cross(pe - pi);
    Vector3d Jo = z;

    J.block<3, 1>(0, i) = Jp;
    J.block<3, 1>(3, i) = Jo;
  }
}

void PandaRobot::getJacobian_world(const Vector7d& q, MatrixXd& J) const {
  MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;

  J = MatrixXd::Zero(6, njoint_);

  Matrix4d T = T_base_;
  std::vector<Matrix4d> T_all(DH.rows(), Matrix4d::Identity());

  for (int i = 0; i < DH.rows(); ++i) {
    Matrix4d Ti;
    Ti << cos(DH(i, 0)), -sin(DH(i, 0)), 0, DH(i, 2),
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)), -DH(i, 1) * sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1) * cos(DH(i, 3)),
          0, 0, 0, 1;
    T *= Ti;
    T_all[i] = T;
  }
  for (int i = 0; i < njoint_; ++i) {
    Matrix4d T_prev = T_all[i];
    Vector3d z = T_prev.block<3, 1>(0, 2);
    Vector3d pe = T.block<3, 1>(0, 3);
    Vector3d pi = T_prev.block<3, 1>(0, 3);
    Vector3d Jp = z.cross(pe - pi);
    Vector3d Jo = z;

    J.block<3, 1>(0, i) = Jp;
    J.block<3, 1>(3, i) = Jo;
  }
}

void PandaRobot::getCollisionSpheres(const Vector7d& q, std::vector<CollisionSphere>& collision_spheres_current) const {
  Eigen::MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;
  int nlink = DH.rows();
  collision_spheres_current.clear();

  Matrix4d T = T_base_;

  for (int i = 0; i < nlink; ++i) {
    Matrix4d Ti;
    Ti << cos(DH(i, 0)), -sin(DH(i, 0)), 0, DH(i, 2),
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)), -DH(i, 1) * sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1) * cos(DH(i, 3)),
          0, 0, 0, 1;
    T *= Ti;

    for (const auto& sphere : collision_spheres_[i]) {
      Vector3d p = T.block<3, 3>(0, 0) * sphere.first + T.block<3, 1>(0, 3);
      collision_spheres_current.emplace_back(p, sphere.second);
    }
  }
  if (attached_spheres_.size() > 0) {
    for (const auto& sphere : attached_spheres_) {
      Vector3d p = T.block<3, 3>(0, 0) * sphere.first + T.block<3, 1>(0, 3);
      collision_spheres_current.emplace_back(p, sphere.second);
    }
  }
}

void PandaRobot::getCollisionSphereJacobian(const Vector7d& q, MatrixXd& J, int sphere_index) const {
  J = MatrixXd::Zero(3, njoint_);
  if (sphere_index > n_collision_spheres_ + n_attached_spheres_) {
    std::cerr << "Sphere index out of range" << std::endl;
    return;
  }
  bool is_attached = (sphere_index > n_collision_spheres_);
  int link_index = 0;
  Vector3d center;
  if (!is_attached) {
    int sphere_count = 0, sphere_index_in_link = 0;
    for (int i = 0; i < (int)collision_spheres_.size(); ++i) {
      if (sphere_count + (int)collision_spheres_[i].size() >= sphere_index) {
        link_index = i;
        sphere_index_in_link = sphere_index - sphere_count - 1;
        center = collision_spheres_[i][sphere_index_in_link].first;
        break;
      }
      sphere_count += collision_spheres_[i].size();
    }
  } else {
    link_index = DH_.rows() - 1;
    int idx = sphere_index - n_collision_spheres_ - 1;
    center = attached_spheres_[idx].first;
  }

  MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;
  Matrix4d T = T_base_;
  std::vector<Matrix4d> T_all(link_index + 1, Matrix4d::Identity());
  for (int i = 0; i < link_index + 1; ++i) {
    Matrix4d Ti;
    Ti << cos(DH(i, 0)), -sin(DH(i, 0)), 0, DH(i, 2),
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)), -DH(i, 1) * sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1) * cos(DH(i, 3)),
          0, 0, 0, 1;
    T = T * Ti;
    T_all[i] = T;
  }
  Matrix4d T_sphere = Matrix4d::Identity();
  T_sphere.block<3, 1>(0, 3) = center;
  T = T * T_sphere;
  for (int i = 0; i < std::min(link_index + 1, njoint_); ++i) {
    Matrix4d T_prev = T_all[i];
    Vector3d z = T_prev.block<3, 1>(0, 2);
    Vector3d pe = T.block<3, 1>(0, 3);
    Vector3d pi = T_prev.block<3, 1>(0, 3);
    Vector3d Jp = z.cross(pe - pi);
    J.block<3, 1>(0, i) = Jp;
  }
}

void PandaRobot::getCollisionSpheres_Jacobians(const Vector7d& q,
                                               std::vector<CollisionSphere>& collision_spheres_current,
                                               std::vector<MatrixXd>& Jacobians) const {
  collision_spheres_current.clear();
  Jacobians.clear();

  MatrixXd DH = DH_;
  DH.col(0).head(njoint_) = q;
  int nlink = DH.rows();

  Matrix4d T = T_base_;
  std::vector<Matrix4d> T_all(nlink, Matrix4d::Identity());

  Matrix4d T_sphere, T_sphere_world, Ti;
  Vector3d z, pe, pi, Jpk;
  for (int i = 0; i < nlink; ++i) {
    Ti << cos(DH(i, 0)), -sin(DH(i, 0)), 0, DH(i, 2),
          sin(DH(i, 0)) * cos(DH(i, 3)), cos(DH(i, 0)) * cos(DH(i, 3)), -sin(DH(i, 3)), -DH(i, 1) * sin(DH(i, 3)),
          sin(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 0)) * sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1) * cos(DH(i, 3)),
          0, 0, 0, 1;
    T *= Ti;
    T_all[i] = T;
    for (const auto& sphere : collision_spheres_[i]) {
      T_sphere = Matrix4d::Identity();
      T_sphere.block<3, 1>(0, 3) = sphere.first;
      T_sphere_world = T * T_sphere;
      Vector3d p_sphere_world = T_sphere_world.block<3, 1>(0, 3);
      MatrixXd J_sphere_world = MatrixXd::Zero(3, njoint_);
      for (int k = 0; k < std::min(njoint_, i + 1); k++) {
        z = T_all[k].block<3, 1>(0, 2);
        pe = T_sphere_world.block<3, 1>(0, 3);
        pi = T_all[k].block<3, 1>(0, 3);
        Jpk = z.cross(pe - pi);
        J_sphere_world.block<3, 1>(0, k) = Jpk;
      }
      Jacobians.push_back(J_sphere_world);
      collision_spheres_current.emplace_back(p_sphere_world, sphere.second);
    }
  }
  if (!attached_spheres_.empty()) {
    for (const auto& sphere : attached_spheres_) {
      T_sphere = Matrix4d::Identity();
      T_sphere.block<3, 1>(0, 3) = sphere.first;
      T_sphere_world = T * T_sphere;
      Vector3d p_sphere_world = T_sphere_world.block<3, 1>(0, 3);
      MatrixXd J_sphere_world = MatrixXd::Zero(3, njoint_);
      for (int k = 0; k < njoint_; k++) {
        z = T_all[k].block<3, 1>(0, 2);
        pe = T_sphere_world.block<3, 1>(0, 3);
        pi = T_all[k].block<3, 1>(0, 3);
        Jpk = z.cross(pe - pi);
        J_sphere_world.block<3, 1>(0, k) = Jpk;
      }
      Jacobians.push_back(J_sphere_world);
      collision_spheres_current.emplace_back(p_sphere_world, sphere.second);
    }
  }
}

double PandaRobot::getManipIndex(const Vector7d& q) const {
  MatrixXd J;
  getJacobian(q, J);
  Eigen::JacobiSVD<MatrixXd> svd(J);
  Eigen::VectorXd singularValues = svd.singularValues();
  return singularValues.prod();
}

void PandaRobot::getManipIndexAndGrad(const Vector7d& q, double& manipIndex, Vector7d& grad) const {
  manipIndex = getManipIndex(q);
  const double epsilon = 1e-5;
  for (int i = 0; i < q.size(); ++i) {
    Vector7d q_perturbed = q;
    q_perturbed(i) += epsilon;
    double manipIndex_perturbed = getManipIndex(q_perturbed);
    grad(i) = (manipIndex_perturbed - manipIndex) / epsilon;
  }
}

void relativePose(const PandaRobot& robot1,
                  const PandaRobot& robot2,
                  const Vector7d& q1,
                  const Vector7d& q2,
                  Matrix4d& T_rel) {
  Matrix4d T1, T2;
  robot1.getT(q1, T1);
  robot2.getT(q2, T2);
  T_rel = T1.inverse() * T2;
}

void relativeJacobian(const PandaRobot& robot1,
                      const PandaRobot& robot2,
                      const Vector7d& q1,
                      const Vector7d& q2,
                      MatrixXd& Jrel) {
  int njoint = robot1.njoint();
  Matrix4d T1, T2;
  robot1.getT(q1, T1);
  robot2.getT(q2, T2);

  MatrixXd J1, J2;
  robot1.getJacobian(q1, J1);
  robot2.getJacobian(q2, J2);

  Matrix4d T_1_2 = T1.inverse() * T2;
  Matrix3d R_1_2 = T_1_2.block<3, 3>(0, 0);
  Vector3d p_1_2 = T_1_2.block<3, 1>(0, 3);

  MatrixXd Psi = MatrixXd::Identity(6, 6);
  Psi.block<3, 3>(0, 3) = -skew(p_1_2);

  Matrix3d R_e1_b1 = T1.block<3, 3>(0, 0).transpose() * robot1.baseRotation();
  MatrixXd Omega1 = MatrixXd::Zero(6, 6);
  Omega1.block<3, 3>(0, 0) = R_e1_b1;
  Omega1.block<3, 3>(3, 3) = R_e1_b1;

  Matrix3d R_e1_b2 = T1.block<3, 3>(0, 0).transpose() * robot2.baseRotation();
  MatrixXd Omega2 = MatrixXd::Zero(6, 6);
  Omega2.block<3, 3>(0, 0) = R_e1_b2;
  Omega2.block<3, 3>(3, 3) = R_e1_b2;

  Jrel = MatrixXd::Zero(6, 2 * njoint);
  Jrel.block(0, 0, 6, njoint) = -Psi * Omega1 * J1;
  Jrel.block(0, njoint, 6, njoint) = Omega2 * J2;
}

Matrix3d eul2Rotm(const Vector3d& eul) {
  return AngleAxisd(eul[0], Vector3d::UnitX()).toRotationMatrix() * AngleAxisd(eul[1], Vector3d::UnitY()).toRotationMatrix() *
         AngleAxisd(eul[2], Vector3d::UnitZ()).toRotationMatrix();
}

Matrix3d skew(const Vector3d& v) {
  Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

} // namespace franka_example_controllers
