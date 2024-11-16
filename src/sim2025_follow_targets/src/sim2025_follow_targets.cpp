#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "sim2025_interfaces/action/follow_target.hpp"
#include "sim2025_interfaces/action/follow_targets.hpp"

std::vector<sim2025_interfaces::action::FollowTarget::Goal> ParseFollowGoalsFromFile(const char *filename) {
    std::vector<sim2025_interfaces::action::FollowTarget::Goal> goals;
    std::ifstream file(filename);
    std::string line;
    while(std::getline(file, line)) {
        int i = 0;
        sim2025_interfaces::action::FollowTarget::Goal the_goal;
        std::string token;
        std::stringstream sstream(line);
        while(std::getline(sstream, token, ',') && i < 2) {
            if(i == 0) {
                the_goal.x = std::stof(token);
            } else if(i == 1) {
                the_goal.y = std::stof(token);
            }
            i++;
        }
        goals.push_back(the_goal);
    }
    return goals;
}

class Sim2025FollowTargets : public rclcpp::Node {
public:
    __attribute__ ((visibility("default")))
    explicit Sim2025FollowTargets(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
private:
    using FollowTarget = sim2025_interfaces::action::FollowTarget;
    using FollowTargets = sim2025_interfaces::action::FollowTargets;
    using GoalHandleFollowTarget = rclcpp_action::ClientGoalHandle<FollowTarget>;
    using GoalHandleFollowTargets = rclcpp_action::ServerGoalHandle<FollowTargets>;

    rclcpp_action::Client<FollowTarget>::SharedPtr follow_target_client;
    rclcpp_action::Server<FollowTargets>::SharedPtr follow_targets_actionsrv;

    std::vector<FollowTarget::Goal> target_coords;

    rclcpp_action::GoalStatus follow_target_goalstatus;

    void follow_target_client_send_goal(FollowTarget::Goal goal);
    void follow_target_response_callback(const GoalHandleFollowTarget::SharedPtr &goal_handle);
    void follow_target_result_callback(const GoalHandleFollowTarget::WrappedResult & result);

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowTargets::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowTargets> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFollowTargets> goal_handle);
    void action_execute(const std::shared_ptr<GoalHandleFollowTargets> goal_handle);
};

Sim2025FollowTargets::Sim2025FollowTargets(const rclcpp::NodeOptions &options) 
: rclcpp::Node("sim2025_follow_targets", options) {
    using namespace std::placeholders;
    follow_target_goalstatus.status = rclcpp_action::GoalStatus::STATUS_EXECUTING;
    follow_target_client = rclcpp_action::create_client<FollowTarget>(
        this,
        "follow_target"
    );
    follow_targets_actionsrv = rclcpp_action::create_server<FollowTargets>(
        this,
        "follow_targets",
        std::bind(&Sim2025FollowTargets::handle_goal, this, _1, _2),
        std::bind(&Sim2025FollowTargets::handle_cancel, this, _1),
        std::bind(&Sim2025FollowTargets::handle_accepted, this, _1)
    );
}

void Sim2025FollowTargets::follow_target_client_send_goal(FollowTarget::Goal goal) {
    using namespace std::placeholders;
    if(!follow_target_client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server /follow_target not available after waiting");
        exit(0);
    }
    follow_target_goalstatus.status = rclcpp_action::GoalStatus::STATUS_EXECUTING;
    RCLCPP_INFO(this->get_logger(), "Sending goal (%f, %f)", goal.x, goal.y);
    auto send_goal_options = rclcpp_action::Client<FollowTarget>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &Sim2025FollowTargets::follow_target_response_callback, this, _1
    );
    send_goal_options.result_callback = std::bind(
        &Sim2025FollowTargets::follow_target_result_callback, this, _1
    );
    follow_target_client->async_send_goal(goal, send_goal_options);
}
void Sim2025FollowTargets::follow_target_response_callback(const GoalHandleFollowTarget::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    
}
void Sim2025FollowTargets::follow_target_result_callback(const GoalHandleFollowTarget::WrappedResult & result) {
    switch(result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED :
        follow_target_goalstatus.status = rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
        break;
    case rclcpp_action::ResultCode::CANCELED :
        follow_target_goalstatus.status = rclcpp_action::GoalStatus::STATUS_CANCELED;
        break;
    case rclcpp_action::ResultCode::ABORTED :
        follow_target_goalstatus.status = rclcpp_action::GoalStatus::STATUS_ABORTED;
        break;
    default:
        follow_target_goalstatus.status = rclcpp_action::GoalStatus::STATUS_UNKNOWN;
        break;
    }
}

rclcpp_action::GoalResponse Sim2025FollowTargets::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowTargets::Goal> goal) {
    try {
        target_coords = ParseFollowGoalsFromFile(goal->buffer_or_filename.c_str());
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Sim2025FollowTargets::handle_cancel(const std::shared_ptr<GoalHandleFollowTargets> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Sim2025FollowTargets::handle_accepted(const std::shared_ptr<GoalHandleFollowTargets> goal_handle) {
    using namespace std::placeholders;
    std::thread(std::bind(&Sim2025FollowTargets::action_execute, this, _1), goal_handle).detach();
}

void Sim2025FollowTargets::action_execute(const std::shared_ptr<GoalHandleFollowTargets> goal_handle) {
    rclcpp::Rate loop_rate(60);

    auto result = std::make_shared<FollowTargets::Result>();

    for(size_t i = 0; i < target_coords.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Sending goal (%f, %f) from index %zu", target_coords[i].x, target_coords[i].y, i);
        follow_target_client_send_goal(target_coords[i]);
        while(follow_target_goalstatus.status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
            loop_rate.sleep();
        }
        if(follow_target_goalstatus.status != rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
            break;
        }
    }
    if(rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Execution Finished");
        goal_handle->succeed(result);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim2025FollowTargets>());
    rclcpp::shutdown();
    
    printf("hello world sim2025_follow_targets package\n");
    return 0;
}
