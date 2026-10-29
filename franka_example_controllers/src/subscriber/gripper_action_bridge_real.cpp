// Real-robot gripper bridge -- "panda style" command scheduling.
//
// gello_software drives the Franka hand from inside its own control loop
// (gello/robots/panda.py::command_joint_state):
//
//     self.gripper.goto(width=MAX_OPEN * (1 - joint_state[-1]), speed=1, force=1)
//
// i.e. exactly one *blocking* command per control step: the next command is only
// issued after the previous one has finished, and every command carries a force
// limit.
//
// franka_msgs Move/Grasp are one-shot trajectories, NOT position setpoints: the
// gripper firmware aborts an in-flight command as soon as a new goal arrives.
// The GELLO publisher streams width_desired at ~50 Hz, so forwarding every
// sample as an asynchronous goal (the previous implementation) meant every
// command was preempted during its acceleration phase and the fingers never
// moved. The simulation hid this because gripper_sim_action_server accepts
// concurrent goals.
//
// This node reproduces the panda.py semantics on the ROS side:
//   * subscription callbacks only latch the newest sample (latest wins, exactly
//     like the next control tick in panda.py); the GELLO publisher is never
//     blocked, so the 50 Hz arm stream keeps running,
//   * a single worker thread sends one command, waits for its result and only
//     then sends the next one, so goals never overlap,
//   * width setpoints are sent as Grasp(width, speed, force), the equivalent of
//     goto(..., force=...), so the hand can hold objects instead of stalling,
//   * goal responses and results are inspected and logged instead of being
//     silently dropped.
//
// Parameters (all writable at runtime, e.g. `ros2 param set <node> speed 0.1`;
// they affect the next command):
//   arm_id           string  "left"   -> topics/actions <arm_id>_gripper/...
//                                     (read at startup, changing it later has no effect)
//   speed            double  1.0      closing speed [m/s] (franka clamps to 0.1)
//   force            double  10.0     grasping force [N] for width setpoints
//                                     (panda.py passes force=1; raise it if the
//                                     hand cannot hold the object)
//   epsilon_inner    double  0.005    [m] grasp tolerance (narrower than target)
//   epsilon_outer    double  0.005    [m] grasp tolerance (wider than target)
//   min_change       double  0.0005   ignore width changes smaller than this [m]
//   max_width        double  0.08     clamp for the commanded width [m]
//   use_grasp        bool    true     true: Grasp (with force), false: Move
//   auto_home        bool    true     home the hand before accepting commands
//   command_timeout  double  5.0      [s] per command, <= 0 disables
//   home_timeout     double  10.0     [s] for the startup homing

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

using MoveAction = franka_msgs::action::Move;
using GraspAction = franka_msgs::action::Grasp;
using HomingAction = franka_msgs::action::Homing;
using TriggerService = std_srvs::srv::Trigger;

class GripperActionBridge : public rclcpp::Node
{
public:
    explicit GripperActionBridge(const std::string& node_name) : Node(node_name)
    {
        // Parameters. arm_id selects the topic/action prefix (<arm_id>_gripper)
        arm_id_ = this->declare_parameter<std::string>("arm_id", "left");
        speed_ = this->declare_parameter<double>("speed", 1.0);
        force_ = this->declare_parameter<double>("force", 10.0);
        epsilon_inner_ = this->declare_parameter<double>("epsilon_inner", 0.005);
        epsilon_outer_ = this->declare_parameter<double>("epsilon_outer", 0.005);
        min_change_ = this->declare_parameter<double>("min_change", 5e-4);
        max_width_ = this->declare_parameter<double>("max_width", 0.08);
        use_grasp_ = this->declare_parameter<bool>("use_grasp", true);
        auto_home_ = this->declare_parameter<bool>("auto_home", true);
        command_timeout_ = this->declare_parameter<double>("command_timeout", 5.0);
        home_timeout_ = this->declare_parameter<double>("home_timeout", 10.0);

        const auto node_prefix = arm_id_ + "_gripper";

        // Topic Names
        auto sub_topic_move = arm_id_ + "_gripper/width_desired";
        auto sub_topic_grasp = arm_id_ + "_gripper/grasp_desired";
        auto sub_topic_homing = arm_id_ + "_gripper/homing_desired";
        auto sub_topic_stop = arm_id_ + "_gripper/stop_desired";

        // Action / Service Names (kept as members for diagnostics)
        move_action_name_ = node_prefix + "/move";
        grasp_action_name_ = node_prefix + "/grasp";
        homing_action_name_ = node_prefix + "/homing";
        stop_service_name_ = node_prefix + "/stop";

        // Subscribers. They only latch the newest command; the worker thread owned
        // by this node performs the blocking action calls (see command_loop()).
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
        move_client_ = rclcpp_action::create_client<MoveAction>(this, move_action_name_);
        grasp_client_ = rclcpp_action::create_client<GraspAction>(this, grasp_action_name_);
        homing_client_ = rclcpp_action::create_client<HomingAction>(this, homing_action_name_);

        // Service Client
        stop_client_ = this->create_client<TriggerService>(stop_service_name_);

        // Runtime tuning: `ros2 param set /<node> force 5.0` affects the next command.
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&GripperActionBridge::on_parameters_set, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Panda-style gripper bridge started for %s arm "
                    "(grasp=%s, speed=%.3f m/s, force=%.1f N, min_change=%.4f m, auto_home=%s).",
                    arm_id_.c_str(), use_grasp_ ? "yes" : "no", speed_, force_, min_change_,
                    auto_home_ ? "yes" : "no");

        // The worker thread is the single owner of the gripper action clients, which
        // is what makes the command sequence "one at a time" like panda.py's goto().
        worker_ = std::thread(&GripperActionBridge::command_loop, this);
    }

    ~GripperActionBridge() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    /// One command executed by the worker thread. Width setpoints carry the
    /// bridge defaults, explicit requests carry their own arguments.
    struct Command
    {
        enum class Type { kWidth, kGrasp, kHoming, kStop };
        Type type = Type::kWidth;
        double width = 0.0;
        double speed = 0.0;
        double force = 0.0;
        double epsilon_inner = 0.005;
        double epsilon_outer = 0.005;
    };

    // ---------------------------------------------------------------------
    // Subscription callbacks. They never block and never call the action
    // clients directly: they only latch the newest sample (width_desired) or
    // queue a discrete request (grasp/homing). This mirrors panda.py, where
    // the control loop merely computes the width that the next goto() uses.
    // ---------------------------------------------------------------------

    void move_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_width_ = msg->data;
        has_new_width_ = true;
        cv_.notify_one();
    }

    void grasp_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        Command command;
        command.type = Command::Type::kGrasp;
        command.speed = speed_;
        command.force = force_;
        command.epsilon_inner = epsilon_inner_;
        command.epsilon_outer = epsilon_outer_;

        // Parse array, expected order: [width, speed, force, epsilon_inner, epsilon_outer]
        if (msg->data.size() > 0) command.width = msg->data[0];
        if (msg->data.size() > 1) command.speed = msg->data[1];
        if (msg->data.size() > 2) command.force = msg->data[2];
        if (msg->data.size() > 3) command.epsilon_inner = msg->data[3];
        if (msg->data.size() > 4) command.epsilon_outer = msg->data[4];

        enqueue(command);
    }

    void homing_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            Command command;
            command.type = Command::Type::kHoming;
            enqueue(command);
        }
    }

    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            return;
        }
        // Stop must not queue behind a blocking command: the service aborts the
        // motion that is currently running, so it is called right here.
        send_stop();
    }

    void enqueue(const Command& command)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_.size() >= kMaxPending) {
            pending_.pop_front();
            RCLCPP_WARN(this->get_logger(), "Gripper command queue full, dropping the oldest request.");
        }
        pending_.push_back(command);
        cv_.notify_one();
    }

    // ---------------------------------------------------------------------
    // Worker thread: the panda.py "one blocking goto() per control step" loop.
    // ---------------------------------------------------------------------

    void command_loop()
    {
        if (auto_home_) {
            // A freshly connected Franka hand reports max_width == 0 until it has
            // been homed, so every move/grasp would fail. panda.py gets a homed
            // gripper from polymetis; we have to ask for it explicitly. The gripper
            // node may still be starting up, so wait for it first.
            if (wait_for_action_server(homing_client_, home_timeout_)) {
                Command homing;
                homing.type = Command::Type::kHoming;
                execute(homing, home_timeout_);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Homing action server '%s' did not become available, the gripper stays "
                             "unhomed and move/grasp will fail. Publish %s_gripper/homing_desired once "
                             "the gripper node is up.",
                             homing_action_name_.c_str(), arm_id_.c_str());
            }
        }

        while (rclcpp::ok()) {
            Command command;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(50), [this] {
                    return shutdown_ || !pending_.empty() || has_new_width_;
                });
                if (shutdown_) {
                    break;
                }
                if (!pending_.empty()) {
                    // Discrete requests (explicit grasp, homing) win over teleop
                    // setpoints so automation commands are not dropped.
                    command = pending_.front();
                    pending_.pop_front();
                } else if (has_new_width_) {
                    // Latest sample wins: older teleop samples are skipped exactly
                    // like a control loop that is still blocked inside goto().
                    command.type = Command::Type::kWidth;
                    command.width = latest_width_;
                    has_new_width_ = false;
                } else {
                    continue;
                }
            }

            if (command.type == Command::Type::kWidth) {
                command.width = std::clamp(command.width, 0.0, max_width_);
                if (resend_next_width_.exchange(false)) {
                    // A stop or homing interrupted the previous motion: send the
                    // current sample even if it matches the last target.
                    last_width_command_ = std::numeric_limits<double>::quiet_NaN();
                }
                if (std::fabs(command.width - last_width_command_) < min_change_) {
                    continue;
                }
                // Remember the target even if the move fails, otherwise a blocked
                // target would be retried forever at the publisher rate.
                last_width_command_ = command.width;
                command.speed = speed_;
                command.force = force_;
                command.epsilon_inner = epsilon_inner_;
                command.epsilon_outer = epsilon_outer_;
            }

            execute(command, command_timeout_);
        }
    }

    // ---------------------------------------------------------------------
    // Blocking dispatch: every call returns only after the action server has
    // finished the motion, which is what keeps the goals from aborting each
    // other (panda.py blocks in goto() in the same way).
    // ---------------------------------------------------------------------

    void execute(const Command& command, double timeout)
    {
        switch (command.type) {
            case Command::Type::kWidth:
                if (use_grasp_) {
                    send_grasp("width", command.width, command.speed, command.force,
                               command.epsilon_inner, command.epsilon_outer, timeout);
                } else {
                    send_move(command.width, command.speed, timeout);
                }
                break;
            case Command::Type::kGrasp:
                send_grasp("grasp", command.width, command.speed, command.force,
                           command.epsilon_inner, command.epsilon_outer, timeout);
                break;
            case Command::Type::kHoming:
                send_homing(timeout);
                break;
            case Command::Type::kStop:
                send_stop();
                break;
        }
    }

    /// Grasp(width, speed, force, epsilon) -- the ROS equivalent of panda.py's
    /// gripper.goto(width=..., speed=..., force=...).
    void send_grasp(const std::string& tag, double width, double speed, double force,
                    double epsilon_inner, double epsilon_outer, double timeout)
    {
        if (!grasp_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Grasp action server '%s' is not available.",
                                 grasp_action_name_.c_str());
            return;
        }

        GraspAction::Goal goal;
        goal.width = width;
        goal.speed = speed;
        goal.force = force;
        goal.epsilon.inner = epsilon_inner;
        goal.epsilon.outer = epsilon_outer;

        std::ostringstream label_stream;
        label_stream << tag << " width=" << width << " m, force=" << force << " N";
        const std::string label = label_stream.str();

        const auto goal_handle = send_goal_blocking<GraspAction>(grasp_client_, goal, timeout, label);
        if (!goal_handle) {
            return;
        }
        const auto result = await_result_blocking<GraspAction>(grasp_client_, goal_handle, timeout, label);
        if (!result) {
            return;
        }
        report_result(label, result->code, result->result != nullptr && result->result->success,
                      result->result ? result->result->error : std::string());
    }

    /// Move(width, speed) -- used only when use_grasp is false.
    void send_move(double width, double speed, double timeout)
    {
        if (!move_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Move action server '%s' is not available.",
                                 move_action_name_.c_str());
            return;
        }

        MoveAction::Goal goal;
        goal.width = width;
        goal.speed = speed;

        std::ostringstream label_stream;
        label_stream << "move width=" << width << " m";
        const std::string label = label_stream.str();

        const auto goal_handle = send_goal_blocking<MoveAction>(move_client_, goal, timeout, label);
        if (!goal_handle) {
            return;
        }
        const auto result = await_result_blocking<MoveAction>(move_client_, goal_handle, timeout, label);
        if (!result) {
            return;
        }
        report_result(label, result->code, result->result != nullptr && result->result->success,
                      result->result ? result->result->error : std::string());
    }

    /// Homing -- must run before the first move/grasp on a freshly connected
    /// hand, otherwise libfranka rejects every command because max_width is 0.
    void send_homing(double timeout)
    {
        if (!homing_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Homing action server '%s' is not available.",
                                 homing_action_name_.c_str());
            return;
        }

        const std::string label = "homing";
        const auto goal_handle =
            send_goal_blocking<HomingAction>(homing_client_, HomingAction::Goal(), timeout, label);
        if (goal_handle) {
            const auto result =
                await_result_blocking<HomingAction>(homing_client_, goal_handle, timeout, label);
            if (result) {
                const bool success = result->code == rclcpp_action::ResultCode::SUCCEEDED &&
                                     result->result != nullptr && result->result->success;
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Gripper homing finished, maximum width is now known.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Gripper homing failed: %s",
                                 result->result ? result->result->error.c_str() : "no result");
                }
            }
        }
        // Homing re-estimates the finger width, so the next width sample has to be
        // sent even if it equals the last commanded value.
        last_width_command_ = std::numeric_limits<double>::quiet_NaN();
    }

    void send_stop()
    {
        if (!stop_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Stop service '%s' is not available.",
                                 stop_service_name_.c_str());
            return;
        }
        auto request = std::make_shared<TriggerService::Request>();
        stop_client_->async_send_request(
            request, [this](rclcpp::Client<TriggerService>::SharedFuture future) {
                const auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Gripper stopped.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Gripper could not be stopped: %s",
                                response->message.c_str());
                }
                // The motion was aborted, so teleop must re-send the current sample
                // and the abort is expected rather than an object being detected.
                stop_requested_ = response->success;
                resend_next_width_ = true;
            });
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    // Runtime parameters
    // ---------------------------------------------------------------------

    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
            const auto& name = parameter.get_name();
            if (name == "speed") {
                speed_ = parameter.as_double();
            } else if (name == "force") {
                force_ = parameter.as_double();
            } else if (name == "epsilon_inner") {
                epsilon_inner_ = parameter.as_double();
            } else if (name == "epsilon_outer") {
                epsilon_outer_ = parameter.as_double();
            } else if (name == "min_change") {
                min_change_ = std::max(0.0, parameter.as_double());
            } else if (name == "max_width") {
                max_width_ = std::max(0.0, parameter.as_double());
            } else if (name == "use_grasp") {
                use_grasp_ = parameter.as_bool();
            } else if (name == "command_timeout") {
                command_timeout_ = parameter.as_double();
            } else if (name == "home_timeout") {
                home_timeout_ = parameter.as_double();
            }
        }
        return result;
    }

    template <typename ActionT>
    bool wait_for_action_server(const std::shared_ptr<rclcpp_action::Client<ActionT>>& client,
                                double timeout)
    {
        const auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !client->action_server_is_ready()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (shutdown_) {
                    return false;
                }
            }
            if (timeout > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >
                                    timeout) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return rclcpp::ok();
    }

    template <typename FutureT>
    bool wait_for_future(FutureT& future, double timeout)
    {
        const auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            if (future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
                return true;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (shutdown_) {
                    return false;
                }
            }
            if (timeout > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >
                                    timeout) {
                return false;
            }
        }
        return false;
    }

    template <typename ActionT>
    std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>
    send_goal_blocking(const typename rclcpp_action::Client<ActionT>::SharedPtr& client,
                       const typename ActionT::Goal& goal, double timeout, const std::string& label)
    {
        auto goal_future = client->async_send_goal(goal);
        if (!wait_for_future(goal_future, timeout)) {
            RCLCPP_WARN(this->get_logger(), "Gripper %s: no goal response within %.1f s.", label.c_str(),
                        timeout);
            return nullptr;
        }
        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Gripper %s: goal rejected by the action server.", label.c_str());
        }
        return goal_handle;
    }

    template <typename ActionT>
    std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult>
    await_result_blocking(const typename rclcpp_action::Client<ActionT>::SharedPtr& client,
                          const std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>& goal_handle,
                          double timeout, const std::string& label)
    {
        auto result_future = client->async_get_result(goal_handle);
        if (!wait_for_future(result_future, timeout)) {
            RCLCPP_WARN(this->get_logger(), "Gripper %s: no result within %.1f s, canceling.", label.c_str(),
                        timeout);
            client->async_cancel_goal(goal_handle);
            return std::nullopt;
        }
        return result_future.get();
    }

    void report_result(const std::string& label, rclcpp_action::ResultCode code, bool success,
                       const std::string& error)
    {
        if (stop_requested_.exchange(false)) {
            // The goal in flight was canceled by the stop service, so a "failed"
            // result here is expected and must not be reported as an error.
            RCLCPP_INFO(this->get_logger(), "Gripper %s interrupted by stop.", label.c_str());
            return;
        }
        if (code == rclcpp_action::ResultCode::SUCCEEDED && success) {
            if (width_stopped_by_object_) {
                RCLCPP_INFO(this->get_logger(), "Gripper %s: target width reachable again.", label.c_str());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Gripper %s: done.", label.c_str());
            }
            width_stopped_by_object_ = false;
            return;
        }
        if (!error.empty()) {
            RCLCPP_WARN(this->get_logger(), "Gripper %s failed: %s", label.c_str(), error.c_str());
            width_stopped_by_object_ = false;
            return;
        }
        if (code == rclcpp_action::ResultCode::ABORTED || code == rclcpp_action::ResultCode::SUCCEEDED) {
            // libfranka returns false without an error message when the fingers stop
            // before reaching the target width, i.e. an object blocked them. That is a
            // normal outcome for a closing command, so report it once and continue.
            if (!width_stopped_by_object_) {
                RCLCPP_INFO(this->get_logger(), "Gripper %s stopped short of the target width (object detected).",
                            label.c_str());
                width_stopped_by_object_ = true;
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Gripper %s stopped short of the target width.", label.c_str());
            }
            return;
        }
        RCLCPP_WARN(this->get_logger(), "Gripper %s ended with result code %d.", label.c_str(),
                    static_cast<int>(code));
    }

    // Parameters
    std::string arm_id_;
    std::string move_action_name_;
    std::string grasp_action_name_;
    std::string homing_action_name_;
    std::string stop_service_name_;
    double speed_{1.0};
    double force_{10.0};
    double epsilon_inner_{0.005};
    double epsilon_outer_{0.005};
    double min_change_{5e-4};
    double max_width_{0.08};
    double command_timeout_{5.0};
    double home_timeout_{10.0};
    bool use_grasp_{true};
    bool auto_home_{true};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Command scheduling shared between the executor thread and the worker thread
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Command> pending_;
    double latest_width_{0.0};
    bool has_new_width_{false};
    bool shutdown_{false};
    std::atomic<bool> resend_next_width_{false};
    std::atomic<bool> stop_requested_{false};

    // Worker-thread only state
    double last_width_command_{std::numeric_limits<double>::quiet_NaN()};
    bool width_stopped_by_object_{false};
    std::thread worker_;

    inline static constexpr std::size_t kMaxPending = 100;

    // Standard ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr move_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grasp_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr homing_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

    // Action and Service Clients (owned by the worker thread)
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
    // Join the command worker (it holds the action clients) before tearing the
    // context down, otherwise an in-flight blocking wait could outlive rclcpp.
    node.reset();
    rclcpp::shutdown();
    return 0;
}
