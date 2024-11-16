#include <iostream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

enum JoyIndexAxis {
    JOY_AXIS_LEFT_X,
    JOY_AXIS_LEFT_Y,
    JOY_AXIS_TRIGGERLEFT,
    JOY_AXIS_RIGHT_X,
    JOY_AXIS_RIGHT_Y,
    JOY_AXIS_TRIGGERRIGHT
};

class Sim2025ControlJoy : public rclcpp::Node  {
public:
    Sim2025ControlJoy();
private:
    double linear_speed;
    double angular_speed;
    float deadzone;
    void joy_subscriber_callback(const sensor_msgs::msg::Joy &joy_msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
    
};

Sim2025ControlJoy::Sim2025ControlJoy() : rclcpp::Node("sim2025_control_joy") , linear_speed(1), angular_speed(M_PI_2) {
    using namespace std::placeholders;
    this->get_parameter("linear_speed", linear_speed);
    this->get_parameter("angular_speed", angular_speed);
    auto joy_param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "joy_node");
    while(!joy_param_client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service, Exiting..");
            rclcpp::shutdown();
            exit(1);
        }
        RCLCPP_INFO(this->get_logger(), "Service /joy_node not available, waiting...");
    }
    deadzone = joy_param_client->get_parameter<float>("deadzone");
    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Sim2025ControlJoy::joy_subscriber_callback, this, _1)
    );
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS())
    );
}

void Sim2025ControlJoy::joy_subscriber_callback(const sensor_msgs::msg::Joy &joy_msg) {
    geometry_msgs::msg::Twist twist;
    if(abs(joy_msg.axes[JOY_AXIS_LEFT_Y]) < deadzone) {
        twist.linear.x = 0;
    } else {
        twist.linear.x = joy_msg.axes[JOY_AXIS_LEFT_Y] * linear_speed;
    }
    if(abs(joy_msg.axes[JOY_AXIS_RIGHT_X]) < deadzone) {
        twist.angular.z = 0;
    } else {
        twist.angular.z = joy_msg.axes[JOY_AXIS_RIGHT_X] * angular_speed;
    }
    twist_publisher->publish(twist);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim2025ControlJoy>());
    rclcpp::shutdown();
    return 0;
}

