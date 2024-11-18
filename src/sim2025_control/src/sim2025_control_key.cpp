#include <iostream>
#include <chrono>
#include <memory>

#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

int getch() {
    int buf;
    termios term, original;
    if(tcgetattr(fileno(stdin), &term) < 0) {
        perror("Oops!, Error getting terminal info");
        exit(1);
    }
    original = term;
    term.c_lflag &= ~ICANON;
    term.c_lflag &= ~ECHO;
    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;
    if(tcsetattr(fileno(stdin), TCSANOW, &term) < 0) {
        perror("Oops!, Error getting terminal info");
        exit(1);
    }
    buf = getchar();
    if(tcsetattr(fileno(stdin), TCSANOW, &original) < 0) {
        perror("Oops!, Error getting terminal info");
        exit(1);
    }
    return buf;
}

class Sim2025ControlKey : public rclcpp::Node  {
public:
    Sim2025ControlKey();
private:
    void timer_callback();
    
    double linear_speed;
    double angular_speed;
    geometry_msgs::msg::Twist current_twist;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
    rclcpp::TimerBase::SharedPtr timer;

};

Sim2025ControlKey::Sim2025ControlKey() : rclcpp::Node("sim2025_control_key") , linear_speed(1), angular_speed(M_PI_2) , current_twist(geometry_msgs::msg::Twist()){
    using namespace std::placeholders;
    this->declare_parameter("linear_speed", 1.0);
    this->declare_parameter("angular_speed", M_PI_2);
    this->get_parameter("linear_speed", linear_speed);
    this->get_parameter("angular_speed", angular_speed);
    timer = this->create_wall_timer(
        10ms,
        std::bind(&Sim2025ControlKey::timer_callback, this)
    );
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS())
    );
    printf(R"(
Welcome to sim2025-control-key
CONTROLS (case-sensitive)
MOVEMENT
    (top->bottom : forward->backward)
    (left->right : counterclockwise->clockwise)

    q   w   e
    a   s   d
    z   x   c
        
initial linear speed : %f, angular speed : %f
)", linear_speed, angular_speed);
}

void Sim2025ControlKey::timer_callback() {
    int input = getch();
    if(input == 3) {
        //interrupt
        timer->cancel();
        return;
    }
    if(input == 'q' || input == 'w' || input == 'e') {
        current_twist.linear.x = linear_speed;
    } else if(input == 'a' || input == 's' || input == 'd') {
        current_twist.linear.x = 0;
    } else if(input == 'z' || input == 'x' || input == 'c') {
        current_twist.linear.x = -linear_speed;
    }

    if(input == 'q' || input == 'a' || input == 'z') {
        current_twist.angular.z = linear_speed;
    } else if (input == 'w' || input == 's' || input == 'x') {
        current_twist.angular.z = 0;
    } else if(input == 'e' || input == 'd' || input == 'c') {
        current_twist.angular.z = -linear_speed;
    }

    twist_publisher->publish(current_twist);
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim2025ControlKey>());
    rclcpp::shutdown();
    return 0;
}

