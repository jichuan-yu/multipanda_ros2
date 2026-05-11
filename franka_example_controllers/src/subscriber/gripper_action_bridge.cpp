#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

using MoveAction = franka_msgs::action::Move;
using GraspAction = franka_msgs::action::Grasp;
using HomingAction = franka_msgs::action::Homing;
using TriggerService = std_srvs::srv::Trigger;

class GripperActionBridge : public rclcpp::Node
{
public:
    GripperActionBridge(const std::string& node_name) : Node(node_name)
    {
        // Parameter for arm selection, could be left or right
        this->declare_parameter<std::string>("arm_id", "left");
        std::string arm_id_ = this->get_parameter("arm_id").as_string();

        auto node_prefix = "/mj_" + arm_id_ + "_gripper_sim_node";
        
        // Topic Names
        auto sub_topic_move = "/mj_" + arm_id_ + "_gripper/width_desired";
        auto sub_topic_grasp = "/mj_" + arm_id_ + "_gripper/grasp_desired";
        auto sub_topic_homing = "/mj_" + arm_id_ + "_gripper/homing_desired";
        auto sub_topic_stop = "/mj_" + arm_id_ + "_gripper/stop_desired";

        // Action / Service Names
        auto action_move = node_prefix + "/move";
        auto action_grasp = node_prefix + "/grasp";
        auto action_homing = node_prefix + "/homing";
        auto service_stop = node_prefix + "/stop";

        // Subscribers
        move_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            sub_topic_move, 10,
            std::bind(&GripperActionBridge::move_callback, this, std::placeholders::_1));

        grasp_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_topic_grasp, 10,
            std::bind(&GripperActionBridge::grasp_callback, this, std::placeholders::_1));

        homing_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            sub_topic_homing, 10,
            std::bind(&GripperActionBridge::homing_callback, this, std::placeholders::_1));

        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            sub_topic_stop, 10,
            std::bind(&GripperActionBridge::stop_callback, this, std::placeholders::_1));

        // Action Clients
        move_client_ = rclcpp_action::create_client<MoveAction>(this, action_move);
        grasp_client_ = rclcpp_action::create_client<GraspAction>(this, action_grasp);
        homing_client_ = rclcpp_action::create_client<HomingAction>(this, action_homing);
        
        // Service Client
        stop_client_ = this->create_client<TriggerService>(service_stop);

        RCLCPP_INFO(this->get_logger(), "Advanced Gripper action bridge started for %s arm.", arm_id_.c_str());
    }

private:
    void move_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!move_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Move action server not available.");
            return;
        }
        auto goal_msg = MoveAction::Goal();
        goal_msg.width = msg->data;
        goal_msg.speed = 0.1;
        auto send_goal_options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
        move_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Forwarded Move command. width: %f", goal_msg.width);
    }

    void grasp_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!grasp_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Grasp action server not available.");
            return;
        }
        
        auto goal_msg = GraspAction::Goal();
        // default values
        goal_msg.width = 0.0;
        goal_msg.speed = 0.1;
        goal_msg.force = 10.0;
        goal_msg.epsilon.inner = 0.005;
        goal_msg.epsilon.outer = 0.005;

        // Parse array, expected order: [width, speed, force, epsilon_inner, epsilon_outer]
        if (msg->data.size() > 0) goal_msg.width = msg->data[0];
        if (msg->data.size() > 1) goal_msg.speed = msg->data[1];
        if (msg->data.size() > 2) goal_msg.force = msg->data[2];
        if (msg->data.size() > 3) goal_msg.epsilon.inner = msg->data[3];
        if (msg->data.size() > 4) goal_msg.epsilon.outer = msg->data[4];

        auto send_goal_options = rclcpp_action::Client<GraspAction>::SendGoalOptions();
        grasp_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Forwarded Grasp command. width: %f, force: %f", goal_msg.width, goal_msg.force);
    }

    void homing_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if(msg->data){
            if (!homing_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
                RCLCPP_WARN(this->get_logger(), "Homing action server not available.");
                return;
            }
            auto goal_msg = HomingAction::Goal();
            auto send_goal_options = rclcpp_action::Client<HomingAction>::SendGoalOptions();
            homing_client_->async_send_goal(goal_msg, send_goal_options);
            RCLCPP_INFO(this->get_logger(), "Forwarded Homing command.");
        }
    }

    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if(msg->data){
            if (!stop_client_->wait_for_service(std::chrono::milliseconds(500))) {
                RCLCPP_WARN(this->get_logger(), "Stop service not available.");
                return;
            }
            auto request = std::make_shared<TriggerService::Request>();
            stop_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Forwarded Stop command.");
        }
    }

    // Standard ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr move_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grasp_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr homing_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

    // Action and Service Clients
    rclcpp_action::Client<MoveAction>::SharedPtr move_client_;
    rclcpp_action::Client<GraspAction>::SharedPtr grasp_client_;
    rclcpp_action::Client<HomingAction>::SharedPtr homing_client_;
    rclcpp::Client<TriggerService>::SharedPtr stop_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionBridge>("gripper_action_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
