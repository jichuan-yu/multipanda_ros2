#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/move.hpp>
#include <string>

using MoveAction = franka_msgs::action::Move;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<MoveAction>;

class GripperActionBridge : public rclcpp::Node
{
public:
    GripperActionBridge(const std::string& node_name) : Node(node_name)
    {
        // Parameter for arm selection, could be left or right
        this->declare_parameter<std::string>("arm_id", "left");
        std::string arm_id_ = this->get_parameter("arm_id").as_string();

        auto sub_topic = "/mj_" + arm_id_ + "_gripper/width_desired";
        auto action_name = "/mj_" + arm_id_ + "_gripper_sim_node/move";

        width_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            sub_topic, 10,
            std::bind(&GripperActionBridge::width_callback, this, std::placeholders::_1));

        action_client_ = rclcpp_action::create_client<MoveAction>(
            this,
            action_name);

        RCLCPP_INFO(this->get_logger(), "Gripper action bridge started for %s arm.", arm_id_.c_str());
    }

private:
    void width_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Action server not available yet.");
            return;
        }

        auto goal_msg = MoveAction::Goal();
        goal_msg.width = msg->data;
        goal_msg.speed = 0.1;

        auto send_goal_options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr width_sub_;
    rclcpp_action::Client<MoveAction>::SharedPtr action_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionBridge>("gripper_action_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
