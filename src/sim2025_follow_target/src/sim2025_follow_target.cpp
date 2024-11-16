#include <cstdlib>
#include <optional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "sim2025_interfaces/action/follow_target.hpp"
#include "sim2025_interfaces/action/follow_targets.hpp"


double get_angle_diff(double a, double b) {
    double diff = b - a;
    while(diff < -M_PI) diff += 2*M_PI;
    while(diff > M_PI) diff -= 2*M_PI;
    return diff;
}

class Sim2025FollowTarget : public rclcpp::Node {
public:
    __attribute__ ((visibility("default")))
    explicit Sim2025FollowTarget(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
private:
    using FollowTarget = sim2025_interfaces::action::FollowTarget;
    using GoalHandleFollowTarget = rclcpp_action::ServerGoalHandle<FollowTarget>;

    std::optional<nav_msgs::msg::Odometry> odom;

    float base_linear_speed;
    float kp;
    float ki;
    float kd;

    void odom_callback(const nav_msgs::msg::Odometry& odom_msg);

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowTarget::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowTarget> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFollowTarget> goal_handle);
    void action_execute(const std::shared_ptr<GoalHandleFollowTarget> goal_handle);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
    rclcpp_action::Server<FollowTarget>::SharedPtr follow_actionsrv;
};

Sim2025FollowTarget::Sim2025FollowTarget(const rclcpp::NodeOptions &options) 
: rclcpp::Node("sim2025_follow_target", options) {
    using namespace std::placeholders;
    std::string node_name_str = this->get_name();
    this->declare_parameter("base_linear_speed", 1.0);
    this->declare_parameter("kp", 4.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 2.0);

    this->get_parameter("base_linear_speed", base_linear_speed);
    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);

    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Sim2025FollowTarget::odom_callback, this, _1)
    );
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS())
    );
    follow_actionsrv = rclcpp_action::create_server<FollowTarget>(
        this,
        "follow_target",
        std::bind(&Sim2025FollowTarget::handle_goal, this, _1, _2),
        std::bind(&Sim2025FollowTarget::handle_cancel, this, _1),
        std::bind(&Sim2025FollowTarget::handle_accepted, this, _1)
    );
}

void Sim2025FollowTarget::odom_callback(const nav_msgs::msg::Odometry& odom_msg) {
    odom = odom_msg;
}

rclcpp_action::GoalResponse Sim2025FollowTarget::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowTarget::Goal> goal) {
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Sim2025FollowTarget::handle_cancel(const std::shared_ptr<GoalHandleFollowTarget> goal_handle) {
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Sim2025FollowTarget::handle_accepted(const std::shared_ptr<GoalHandleFollowTarget> goal_handle) {
    using namespace std::placeholders;
    std::thread(std::bind(&Sim2025FollowTarget::action_execute, this, _1), goal_handle).detach();
}

void Sim2025FollowTarget::action_execute(const std::shared_ptr<GoalHandleFollowTarget> goal_handle) {
    rclcpp::Rate loop_rate(60);
    while(!odom.has_value()) {
        printf("Waiting for odom\n");
    }
    RCLCPP_INFO(this->get_logger(), "Executing Goal");
    auto feedback = std::make_shared<FollowTarget::Feedback>();
    const std::shared_ptr<const FollowTarget::Goal> goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowTarget::Result>();
    geometry_msgs::msg::Pose init_odom_pose = odom.value().pose.pose;
    float last_error = 0;
    float total_error = 0;
    while(true) {
        this->get_parameter("base_linear_speed", base_linear_speed);
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);
        if(goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }
        geometry_msgs::msg::Pose odom_pose = odom.value().pose.pose;
        float pose_yaw = atan2(2 * (odom_pose.orientation.w * odom_pose.orientation.z + odom_pose.orientation.x * odom_pose.orientation.y),
            1 - 2 * (odom_pose.orientation.y * odom_pose.orientation.y + odom_pose.orientation.z * odom_pose.orientation.z)
        );
        
        float distance_to_goal = hypot(goal->x - odom_pose.position.x, goal->y - odom_pose.position.y);
        if(abs(distance_to_goal) < 0.1) {
            twist_publisher->publish(geometry_msgs::msg::Twist());
            break;
        }
        float angle_to_goal = atan2(goal->y - odom_pose.position.y, goal->x - odom_pose.position.x);

        float error = get_angle_diff(pose_yaw, angle_to_goal);
        total_error += error;

        float pid = kp * error + ki * total_error + kd * (error - last_error);
        
        geometry_msgs::msg::Twist twist;
        if(distance_to_goal > base_linear_speed) {
            twist.linear.x = base_linear_speed;
        } else {
            twist.linear.x = distance_to_goal;
        }
        twist.angular.z = pid;
        twist_publisher->publish(twist);

        feedback->err_p = error;
        feedback->err_i = total_error;
        feedback->err_d = error - last_error;
        feedback->pid = pid;

        last_error = error;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    if(rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Goal Reached!");
        goal_handle->succeed(result);
    }
}
RCLCPP_COMPONENTS_REGISTER_NODE(Sim2025FollowTarget)
/*
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim2025FollowTarget>());
    rclcpp::shutdown();
    return 0;
}
*/
