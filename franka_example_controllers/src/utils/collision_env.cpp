#include <iostream>
#include <fstream>
#include <filesystem>
#include "franka_example_controllers/utils/collision_env.h"

CollisionEnv::CollisionEnv()
{
}

void CollisionEnv::loadCollisionObjects(std::string collision_env_config)
{
    collision_objects_.clear();
    if (!std::ifstream(collision_env_config))
    {
        std::cerr << "Collision env config file not found: " << collision_env_config
                  << ". Failed to load collision objects. " << std::endl;
        return;
    }

    YAML::Node config = YAML::LoadFile(collision_env_config);
    const auto &collision_objects = config["collision_objects"];

    if (!collision_objects || collision_objects.size() == 0)
    {
        std::cerr << "No collision objects found in the config file: " << collision_env_config << std::endl;
        return;
    }

    for (const auto &obj : collision_objects)
    {
        std::string id = obj["id"].as<std::string>();
        std::string type = obj["type"].as<std::string>();

        std::vector<double> position = obj["pose"]["position"].as<std::vector<double>>();
        std::vector<double> orientation = obj["pose"]["orientation"].as<std::vector<double>>();

        
        fcl::Transform3<double> transform;
        transform.translation() = fcl::Vector3<double>(position[0], position[1], position[2]);
        transform.linear() = fcl::Quaternion<double>(orientation[3], orientation[0], orientation[1], orientation[2]).toRotationMatrix();
        std::shared_ptr<fcl::CollisionGeometry<double>> geometry;

        if (type == "Box")
        {
            std::vector<double> dimensions = obj["dimensions"].as<std::vector<double>>();
            geometry = std::make_shared<fcl::Box<double>>(dimensions[0], dimensions[1], dimensions[2]);
            fcl::CollisionObject<double> collision_object(geometry, transform);
            collision_objects_.push_back(DynamicObstacle(id,collision_object,Vector3d::Zero()));
        }
        else if (type == "Cylinder")
        {
            std::vector<double> dimensions = obj["dimensions"].as<std::vector<double>>();
            geometry = std::make_shared<fcl::Cylinder<double>>(dimensions[0], dimensions[1]);
            fcl::CollisionObject<double> collision_object(geometry, transform);
            collision_objects_.push_back(DynamicObstacle(id,collision_object,Vector3d::Zero()));
        }
        else if (type == "Sphere")
        {
            double radius = obj["dimensions"][0].as<double>();
            geometry = std::make_shared<fcl::Sphere<double>>(radius);
            fcl::CollisionObject<double> collision_object(geometry, transform);
            collision_objects_.push_back(DynamicObstacle(id,collision_object,Vector3d::Zero()));
        }
        else
        {
            std::cerr << "Unknown collision object type: " << type << std::endl;
            continue;
        }
    }
    std::cout << "Loaded " << collision_objects_.size() << " collision objects." << std::endl;
}

bool CollisionEnv::isEmpty() const
{
    return collision_objects_.size() == 0;
}

void CollisionEnv::dynamicObstacleHandler(const dual_arm_reactive_control::msg::CollisionObject &msg)
{
    uint8_t operation = msg.operation;
    bool found = false;
    int found_index = -1;

    for (int i = 0; i < collision_objects_.size(); i++)
    {
        if (collision_objects_[i].id == msg.id)
        {
            found = true;
            found_index = i;
            break;
        }
    }
    switch (operation)
    {
    case msg.ADD:
    case msg.MOVE: // same handling logic
        if (found)
        { // update the pose of the object
            fcl::Transform3<double> transform;
            transform.translation() = fcl::Vector3<double>(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
            transform.linear() = fcl::Quaternion<double>(msg.pose.orientation.w, msg.pose.orientation.x,
                                                         msg.pose.orientation.y, msg.pose.orientation.z)
                                     .toRotationMatrix();
            Vector3d velocity(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
           
            collision_objects_[found_index].collision_object.setTransform(transform);
            collision_objects_[found_index].velocity = velocity;
            // std::cout << "Collision Env: Object pose updated: " << msg.id << std::endl;
        }
        else
        { // create a new object
            std::shared_ptr<fcl::CollisionGeometry<double>> geometry;
            Vector3d velocity(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
            if (msg.geometry.type == shape_msgs::msg::SolidPrimitive::SPHERE)
            {
                geometry = std::make_shared<fcl::Sphere<double>>(msg.geometry.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS]);
            }
            else if (msg.geometry.type == shape_msgs::msg::SolidPrimitive::BOX)
            {
                geometry = std::make_shared<fcl::Box<double>>(msg.geometry.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
                                                              msg.geometry.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
                                                              msg.geometry.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);
            }
            else if (msg.geometry.type == shape_msgs::msg::SolidPrimitive::CYLINDER)
            {
                geometry = std::make_shared<fcl::Cylinder<double>>(msg.geometry.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
                                                                   msg.geometry.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]);
            }
            else
            {
                std::cerr << "Unsupported geometry type: " << msg.geometry.type << std::endl;
                return;
            }
            fcl::Transform3<double> transform;
            transform.translation() = fcl::Vector3<double>(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
            transform.linear() = fcl::Quaternion<double>(msg.pose.orientation.w, msg.pose.orientation.x,
                                                         msg.pose.orientation.y, msg.pose.orientation.z)
                                     .toRotationMatrix();
            fcl::CollisionObject<double> collision_object(geometry, transform);
            collision_objects_.push_back(DynamicObstacle(msg.id,collision_object,velocity));
            std::cout << "Collision Env: Object added: " << msg.id << std::endl;
        }
        break;
    case msg.REMOVE:
        if (found)
        {
            collision_objects_.erase(collision_objects_.begin() + found_index);
            std::cout << "Collision Env: Object removed: " << msg.id << std::endl;
        }
        else
        {
            std::cout << "Collision Env: Object to be removed not found: " << msg.id << std::endl;
        }
        break;
    default:
        std::cerr << "Collision Env: Unsupported operation: " << static_cast<int>(operation) << std::endl;
        break;
    }
}

void CollisionEnv::robot2EnvDistanceGradient(const PandaRobot &robot, const Vector7d &q,
                                             double &distance, Vector7d &grad) const
{
    /************ obtain robot collision spheres ************/
    std::vector<CollisionSphere> robot_collision_spheres;
    robot.getCollisionSpheres(q, robot_collision_spheres);

    distance = std::numeric_limits<double>::max();
    grad.setZero();

    if (collision_objects_.size() == 0)
    {
        // std::cout << "No collision objects in the environment." << std::endl;
        return;
    }
    /************ create collision manager for robot and environment ************/
    fcl::BroadPhaseCollisionManagerd *robot_manager = new fcl::DynamicAABBTreeCollisionManagerd();
    fcl::BroadPhaseCollisionManagerd *env_manager = new fcl::DynamicAABBTreeCollisionManagerd();
    std::vector<std::shared_ptr<fcl::CollisionObject<double>>> robot_collision_objects;
    for (int i = 0; i < robot_collision_spheres.size(); i++)
    {
        const auto &sphere = robot_collision_spheres[i];
        std::shared_ptr<fcl::CollisionGeometry<double>> geometry = std::make_shared<fcl::Sphere<double>>(sphere.second);
        fcl::Transform3<double> transform = fcl::Transform3<double>::Identity();
        transform.translation() = fcl::Vector3<double>(sphere.first[0], sphere.first[1], sphere.first[2]);
        auto collision_object = std::make_shared<fcl::CollisionObject<double>>(geometry, transform);
        robot_collision_objects.push_back(collision_object);
        robot_manager->registerObject(collision_object.get());
    }
    for (const auto &obj : collision_objects_)
    {
        env_manager->registerObject(const_cast<fcl::CollisionObject<double> *>(&obj.collision_object));
    }
    /************ compute collision distance ************/
    fcl::DefaultDistanceData<double> distance_data;
    distance_data.request.enable_nearest_points = true;
    distance_data.request.enable_signed_distance = true;
    robot_manager->setup();
    env_manager->setup();
    robot_manager->distance(env_manager, &distance_data, fcl::DefaultDistanceFunction<double>);
    distance = distance_data.result.min_distance;
    Vector3d P1 = distance_data.result.nearest_points[0];                       // nearest point on robot
    Vector3d P2 = distance_data.result.nearest_points[1];                       // nearest point on obstacle
    const fcl::CollisionGeometry<double> *robot_geom = distance_data.result.o1; // nearest object on robot

    /************ find the index of the nearest sphere on robot  ************/
    int sphere_index = 0;
    for (int i = 0; i < robot_collision_objects.size(); i++)
    {
        if (robot_collision_objects[i]->collisionGeometry().get() == robot_geom)
        {
            sphere_index = i;
            break;
        }
    }
    // std::cout << "P1:" << P1.transpose() << std::endl;
    // std::cout << "P2:" << P2.transpose() << std::endl;
    // std::cout << "sphere_index:" << sphere_index << std::endl;

    /************ compute gradient ************/
    Vector3d diff = P1 - P2;
    MatrixXd J_sphere;
    robot.getCollisionSphereJacobian(q, J_sphere, sphere_index + 1); // the index starts from 0
    grad = diff.transpose() / distance * J_sphere;
}

void CollisionEnv::robot2EnvDistanceGradient_Exhaustive(const PandaRobot &robot, const Vector7d &q,
                                                        double &distance, Vector7d &grad) const
{
    distance = std::numeric_limits<double>::max();
    grad.setZero();

    /************ obtain robot collision spheres ************/
    std::vector<CollisionSphere> robot_collision_spheres;
    robot.getCollisionSpheres(q, robot_collision_spheres);

    if (collision_objects_.size() == 0)
    {
        // std::cout << "No collision objects in the environment." << std::endl;
        return;
    }

    int sphere_index = 0;
    Vector3d P1, P2;
    /************ compute collision distance ************/
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    for (int i = 0; i < robot_collision_spheres.size(); i++)
    {
        const auto &sphere = robot_collision_spheres[i];
        std::shared_ptr<fcl::CollisionGeometry<double>> geometry = std::make_shared<fcl::Sphere<double>>(sphere.second);
        fcl::Transform3<double> transform = fcl::Transform3<double>::Identity();
        transform.translation() = fcl::Vector3<double>(sphere.first[0], sphere.first[1], sphere.first[2]);
        auto robot_collision_object = std::make_shared<fcl::CollisionObject<double>>(geometry, transform);

        for (const auto &obj : collision_objects_)
        {
            result.clear(); // must clear the result before next distance computation
            fcl::distance(robot_collision_object.get(), &obj.collision_object, request, result);
            if (result.min_distance < distance)
            {
                distance = result.min_distance;
                P1 = result.nearest_points[0]; // on robot
                P2 = result.nearest_points[1]; // on obstacle
                sphere_index = i;
            }
        }
    }
    // std::cout << "P1:" << P1.transpose() << std::endl;
    // std::cout << "P2:" << P2.transpose() << std::endl;
    // std::cout << "sphere_index:" << sphere_index << std::endl;

    /************ compute gradient ************/
    Vector3d diff = P1 - P2;
    MatrixXd J_sphere;
    robot.getCollisionSphereJacobian(q, J_sphere, sphere_index + 1); // the index starts from 0
    grad = diff.transpose() / distance * J_sphere;
}

void CollisionEnv::robot2EnvDistanceGradient_Pairwise(const PandaRobot &robot, const Vector7d &q,
                                                        VectorXd &distance, MatrixXd &grad) const
{
    if (collision_objects_.empty())
    {
        // std::cout << "No collision objects in the environment." << std::endl;
        distance.resize(1);
        distance.setConstant(std::numeric_limits<double>::max());
        grad.resize(1, 7);
        grad.setZero();
        return;
    }else{
        distance.resize(robot.numSpheres() * collision_objects_.size());
        distance.setConstant(std::numeric_limits<double>::max());
        grad.resize(robot.numSpheres() * collision_objects_.size(), 7);
        grad.setZero();
    }
    /************ obtain robot collision spheres ************/
    std::vector<CollisionSphere> robot_collision_spheres;
    std::vector<MatrixXd> Jacobians;
    robot.getCollisionSpheres_Jacobians(q, robot_collision_spheres, Jacobians);
    int sphere_index = 0;
    Vector3d P1, P2;
    /************ compute collision distance ************/
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    for (int i = 0; i < robot_collision_spheres.size(); i++)
    {
        const auto &sphere = robot_collision_spheres[i];
        std::shared_ptr<fcl::CollisionGeometry<double>> geometry = std::make_shared<fcl::Sphere<double>>(sphere.second);
        fcl::Transform3<double> transform = fcl::Transform3<double>::Identity();
        transform.translation() = fcl::Vector3<double>(sphere.first[0], sphere.first[1], sphere.first[2]);
        auto robot_collision_object = std::make_shared<fcl::CollisionObject<double>>(geometry, transform);

        for (int j=0; j < collision_objects_.size(); j++)
        {
            result.clear(); // must clear the result before next distance computation
            fcl::distance(robot_collision_object.get(), &collision_objects_[j].collision_object, request, result);
            distance(i*collision_objects_.size() + j) = result.min_distance;
            P1 = result.nearest_points[0]; // on robot
            P2 = result.nearest_points[1]; // on obstacle
            Vector3d diff = P1 - P2;
            grad.row(i*collision_objects_.size() + j) = diff.transpose() / result.min_distance * Jacobians[i];
        }
    }
}

/*
    Continuous Differentiable distance approximation using
    LogSumExp-based smooth min function
    **The computation time is about twice as long as the exhaustive method**
*/
void CollisionEnv::robot2EnvDistanceGradient_SMIN(const PandaRobot &robot, const Vector7d &q,
                                            double &distance, double &distance_smin, Vector7d &grad) const
{
    distance = std::numeric_limits<double>::max();
    distance_smin = std::numeric_limits<double>::max();
    grad.setZero();
    /************ obtain robot collision spheres and Jacobians************/
    std::vector<CollisionSphere> robot_collision_spheres;
    std::vector<MatrixXd> Jacobians;
    robot.getCollisionSpheres_Jacobians(q, robot_collision_spheres, Jacobians);

    if (collision_objects_.size() == 0)
    {
        // std::cout << "No collision objects in the environment." << std::endl;
        return;
    }

    /************ Init Parameters ************/
    Vector7d numerator = Vector7d::Zero(); // for gradient computation
    double denominator = 0; // for gradient computation
    double dij = 0;
    double exp_term = 0; 
    Vector3d P1, P2, diff; // normalized distance vector
    Vector7d sphere_grad;
    /************ compute collision distance ************/
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;

    for (int i = 0; i < robot_collision_spheres.size(); i++)
    {
        /* create collision object for fcl distance computation */
        const auto &sphere = robot_collision_spheres[i];
        std::shared_ptr<fcl::CollisionGeometry<double>> geometry = std::make_shared<fcl::Sphere<double>>(sphere.second);
        fcl::Transform3<double> transform = fcl::Transform3<double>::Identity();
        transform.translation() = fcl::Vector3<double>(sphere.first[0], sphere.first[1], sphere.first[2]);
        auto robot_collision_object = std::make_shared<fcl::CollisionObject<double>>(geometry, transform);  

        for (const auto &obj : collision_objects_)
        {
            result.clear(); // must clear the result before next distance computation
            fcl::distance(robot_collision_object.get(), &obj.collision_object, request, result);
            dij = result.min_distance;
            P1 = result.nearest_points[0]; // on robot
            P2 = result.nearest_points[1]; // on obstacle
            if (dij <= 0)
            {
                std::cout << "Collision Env WARNING: distance <= 0, Potential Bug in Gradient Computation!" << std::endl;
            }
            diff = (P1 - P2)/dij; // TODO: distance <= 0 ???
            exp_term = exp(-alpha_ * dij);
            sphere_grad = diff.transpose()*Jacobians[i];

            numerator +=  exp_term*sphere_grad;
            denominator += exp_term;
            distance = std::min(distance, dij);
        }
    }
    distance_smin = -1/alpha_*log(denominator);
    grad = numerator/denominator;

}


/*
    Consider time derivative of the distance (i.e. consider obstacle velocity)
    Continuous Differentiable distance approximation using
    LogSumExp-based smooth min function
*/
void CollisionEnv::robot2EnvDistanceGradient_SMIN_TV(const PandaRobot &robot, const Vector7d &q,
                                        double &distance, double &distance_smin, Vector7d &grad, double &grad_t) const
{
    distance = std::numeric_limits<double>::max();
    distance_smin = std::numeric_limits<double>::max();
    grad.setZero();
    grad_t = 0;
    /************ obtain robot collision spheres and Jacobians************/
    std::vector<CollisionSphere> robot_collision_spheres;
    std::vector<MatrixXd> Jacobians;
    robot.getCollisionSpheres_Jacobians(q, robot_collision_spheres, Jacobians);

    if (collision_objects_.size() == 0)
    {
        // std::cout << "No collision objects in the environment." << std::endl;
        return;
    }

    /************ Init Parameters ************/
    Vector7d numerator = Vector7d::Zero(); // for gradient computation
    double numerator_t = 0; // for time derivative of the gradient
    double denominator = 0; // for gradient computation
    double dij = 0;
    double exp_term = 0; 
    Vector3d P1, P2, diff; 
    Vector7d sphere_grad;
    /************ compute collision distance ************/
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;

    for (int i = 0; i < robot_collision_spheres.size(); i++)
    {
        /* create collision object for fcl distance computation */
        const auto &sphere = robot_collision_spheres[i];
        std::shared_ptr<fcl::CollisionGeometry<double>> geometry = std::make_shared<fcl::Sphere<double>>(sphere.second);
        fcl::Transform3<double> transform = fcl::Transform3<double>::Identity();
        transform.translation() = fcl::Vector3<double>(sphere.first[0], sphere.first[1], sphere.first[2]);
        auto robot_collision_object = std::make_shared<fcl::CollisionObject<double>>(geometry, transform);  

        for (const auto &obj : collision_objects_)
        {
            result.clear(); // must clear the result before next distance computation
            fcl::distance(robot_collision_object.get(), &obj.collision_object, request, result);
            dij = result.min_distance;
            P1 = result.nearest_points[0]; // on robot
            P2 = result.nearest_points[1]; // on obstacle
            if (dij <= 0)
            {
                std::cout << "Collision Env WARNING: distance <= 0, Potential Bug in Gradient Computation!" << std::endl;
            }
            diff = (P1 - P2)/dij; // TODO: distance <= 0 ???
            exp_term = exp(-alpha_ * dij);
            sphere_grad = diff.transpose()*Jacobians[i];

            numerator_t += exp_term*(-diff.dot(obj.velocity));  // 注意这里要取负号
            numerator +=  exp_term*sphere_grad;
            denominator += exp_term;
            distance = std::min(distance, dij);
        }
    }
    distance_smin = -1/alpha_*log(denominator);
    grad = numerator/denominator;
    grad_t = numerator_t/denominator;
}

void robot2RobotDistanceGradient(const PandaRobot &robot1, const Vector7d &q1,
                                 const PandaRobot &robot2, const Vector7d &q2,
                                 double &distance, Vector14d &grad)
{
    std::vector<CollisionSphere> robot1_collision_spheres, robot2_collision_spheres;
    robot1.getCollisionSpheres(q1, robot1_collision_spheres);
    robot2.getCollisionSpheres(q2, robot2_collision_spheres);

    distance = std::numeric_limits<double>::max();
    grad.setZero();

    int min_i = 0, min_j = 0;
    for (int i = 0; i < robot1_collision_spheres.size(); ++i)
    {
        for (int j = 0; j < robot2_collision_spheres.size(); ++j)
        {
            const auto &sphere1 = robot1_collision_spheres[i];
            const auto &sphere2 = robot2_collision_spheres[j];
            Vector3d diff = sphere1.first - sphere2.first;
            double current_distance = diff.norm() - (sphere1.second + sphere2.second);
            if (current_distance < distance)
            {
                distance = current_distance;
                min_i = i;
                min_j = j;
            }
        }
    }
    std::cout << "min_i: " << min_i << " min_j:" << min_j << std::endl;
    MatrixXd J1, J2;
    robot1.getCollisionSphereJacobian(q1, J1, min_i + 1);
    robot2.getCollisionSphereJacobian(q2, J2, min_j + 1);

    Vector3d diff = robot1_collision_spheres[min_i].first - robot2_collision_spheres[min_j].first;
    diff = diff / (distance + robot1_collision_spheres[min_i].second + robot2_collision_spheres[min_j].second);
    grad.head(7) = diff.transpose() * J1;
    grad.tail(7) = -diff.transpose() * J2;
}
