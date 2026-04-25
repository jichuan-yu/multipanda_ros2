/*
    Author: Jichuan Yu
    Date: 2024.11.11
*/
#ifndef COLLISION_ENV_H
#define COLLISION_ENV_H
#include <Eigen/Dense>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <limits>
#include <geometry_msgs/msg/twist.hpp>
#include "franka_example_controllers/utils/robot_kinematics.hpp"
#include "dual_arm_reactive_control/msg/collision_object.hpp" // msg file

using namespace Eigen;
using Vector14d = Eigen::Matrix<double, 14, 1>;

// Import types from franka_example_controllers namespace
using franka_example_controllers::PandaRobot;
using franka_example_controllers::Vector7d;
using franka_example_controllers::CollisionSphere;


struct DynamicObstacle
{
    std::string id;
    fcl::CollisionObjectd collision_object;
    Vector3d velocity;

    DynamicObstacle(const std::string &id, const fcl::CollisionObjectd &collision_object, const Vector3d &velocity)
        : id(id), collision_object(collision_object), velocity(velocity) {}
};

class CollisionEnv
{
public:
    CollisionEnv();

    // load static collision objects from yaml file
    void loadCollisionObjects(std::string collision_env_config);

    // dynamic collision object msg handler
    void dynamicObstacleHandler(const dual_arm_reactive_control::msg::CollisionObject &msg);

    /*
        Fast group to group distance calculation using AABBTree
    */
    void robot2EnvDistanceGradient(const PandaRobot &robot, const Vector7d &q,
                                   double &distance, Vector7d &grad) const;
    /*
        Exhaustive group to group distance calculation
    */
    void robot2EnvDistanceGradient_Exhaustive(const PandaRobot &robot, const Vector7d &q,
                                              double &distance, Vector7d &grad) const;

    /*
        Pair-wise distance calculation
    */
    void robot2EnvDistanceGradient_Pairwise(const PandaRobot &robot, const Vector7d &q,
                                              VectorXd &distance, MatrixXd &grad) const;
    /*
        Continuous Differentiable distance approximation using
        LogSumExp-based smooth min function
    */
    void robot2EnvDistanceGradient_SMIN(const PandaRobot &robot, const Vector7d &q,
                                              double &distance, double &distance_smin, Vector7d &grad) const;


    /*
        Consider time derivative of the distance (i.e. consider obstacle velocity)
        Continuous Differentiable distance approximation using
        LogSumExp-based smooth min function
    */
    void robot2EnvDistanceGradient_SMIN_TV(const PandaRobot &robot, const Vector7d &q,
                                            double &distance, double &distance_smin, Vector7d &grad, double &grad_t) const;

    bool isEmpty() const; // check if the collision environment is empty
private:
    std::vector<DynamicObstacle> collision_objects_; 

    double alpha_ = 100; // smoothness parameter for smooth min function (the smaller, the smoother, but at the cost of lower approximation accuracy)
};

void robot2RobotDistanceGradient(const PandaRobot &robot1, const Vector7d &q1,
                                 const PandaRobot &robot2, const Vector7d &q2,
                                 double &distance, Vector14d &grad);

#endif // COLLISION_ENV_H