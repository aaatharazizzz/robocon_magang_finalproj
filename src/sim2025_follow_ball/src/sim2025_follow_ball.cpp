#include <cstdio>
#include <algorithm>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <sensor_msgs/msg/point_field.hpp>

int min_h = 0;
    int min_s;
    int min_v;
    int max_h;
    int max_s;
    int max_v;
    void on_min_h_thresh_trackbar(int newval, void *) {
    min_h = std::min(179, min_h);
    cv::setTrackbarPos("min_h", "mie_gacoan", newval);
    }


class Sim2025FollowBall : public rclcpp::Node {
public:
    Sim2025FollowBall();
    void Run();

private:
    
    cv::Mat image_frame;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_shared;

    int h_min;
    int s_min;
    int v_min;
    int h_max;
    int s_max;
    int v_max;
    double hough_dp;
    double hough_mindist;
    double hough_prm1;
    double hough_prm2;
    float base_linear_speed;
    float kp;
    float ki;
    float kd;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_publisher;


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
}; 


Sim2025FollowBall::Sim2025FollowBall() : rclcpp::Node("follow_ball") {
    using namespace std::placeholders;
    this->declare_parameter("h_min", 0);
    this->declare_parameter("s_min", 0);
    this->declare_parameter("v_min", 0);
    this->declare_parameter("h_max", 29);
    this->declare_parameter("s_max", 96);
    this->declare_parameter("v_max", 255);
    this->declare_parameter("hough_dp", 1.5);
    this->declare_parameter("hough_mindist", 32.0);
    this->declare_parameter("hough_prm1", 30.0);
    this->declare_parameter("hough_prm2", 80.0);
    this->declare_parameter("base_linear_speed", 1.0);
    this->declare_parameter("kp", 4.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 2.0);


    
    //cv::setTrackbarPos("min_h", "mie_gacoan", )
    //cv::createTrackbar("min_s", "mie_gacoan", &min_s, 255);
    //cv::createTrackbar("min_v", "mie_gacoan", &min_v, 255);
    //cv::createTrackbar("max_h", "mie_gacoan", &max_h, 179);
    //cv::createTrackbar("max_s", "mie_gacoan", &max_s, 255);
    //cv::createTrackbar("max_v", "mie_gacoan", &max_v, 255);
    

    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "depth_camera/image_raw",
        10,
        std::bind(&Sim2025FollowBall::image_callback, this, _1)
    );
    pointcloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "depth_camera/points",
        10,
        std::bind(&Sim2025FollowBall::pointcloud_callback, this, _1)
    );
    cmdvel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10
    );
}

void Sim2025FollowBall::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        image_frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cvbridge error : %s", e.what());
    }
}

void Sim2025FollowBall::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pointcloud_shared = msg;
}



void Sim2025FollowBall::Run() {
    rclcpp::Rate rate(60);

    float last_pid_error = 0;
    float total_pid_error = 0;

    while(rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        this->get_parameter("h_min", h_min);
        this->get_parameter("s_min", s_min);
        this->get_parameter("v_min", v_min);
        this->get_parameter("h_max", h_max);
        this->get_parameter("s_max", s_max);
        this->get_parameter("v_max", v_max);
        this->get_parameter("hough_dp", hough_dp);
        this->get_parameter("hough_mindist", hough_mindist);
        this->get_parameter("hough_prm1", hough_prm1);
        this->get_parameter("hough_prm2", hough_prm2);
        this->get_parameter("base_linear_speed", base_linear_speed);
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);
        if(!image_frame.empty() && pointcloud_shared != nullptr) {
            cv::Mat grayframe;//, mask, canny;
            cv::cvtColor(image_frame, grayframe, cv::COLOR_BGR2GRAY);
            std::vector<cv::Vec3f> circles;
            cv::Mat blurframe;
            cv::GaussianBlur(grayframe, blurframe, cv::Size(11, 11), 0);
            //cv::inRange(image_frame, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);            
            //cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);


            cv::HoughCircles(grayframe, circles, CV_HOUGH_GRADIENT, hough_dp, hough_mindist, hough_prm1, hough_prm2, 10);

            for (cv::Vec3f &circle : circles) {
                cv::circle(image_frame, cv::Point(circle[0], circle[1]), circle[2], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::circle(image_frame, cv::Point(circle[0], circle[1]), 1, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
            }
            
            cv::imshow("Image", image_frame);

            geometry_msgs::msg::Twist twist;

            if(!circles.empty()) {
                float x, y, z;
                cv::Point2i origin = cv::Point2i(circles[0][0], circles[0][1]);
                origin.x = std::min(image_frame.cols, origin.x);
                origin.y = std::min(image_frame.rows, origin.y);
                size_t flat_idx = origin.y * image_frame.cols + origin.x;
                *((uint8_t *)(&x) + 0) = pointcloud_shared->data[flat_idx * 32 + 0];
                *((uint8_t *)(&x) + 1) = pointcloud_shared->data[flat_idx * 32 + 1];
                *((uint8_t *)(&x) + 2) = pointcloud_shared->data[flat_idx * 32 + 2];
                *((uint8_t *)(&x) + 3) = pointcloud_shared->data[flat_idx * 32 + 3];
                *((uint8_t *)(&y) + 0) = pointcloud_shared->data[flat_idx * 32 + 4];
                *((uint8_t *)(&y) + 1) = pointcloud_shared->data[flat_idx * 32 + 5];
                *((uint8_t *)(&y) + 2) = pointcloud_shared->data[flat_idx * 32 + 6];
                *((uint8_t *)(&y) + 3) = pointcloud_shared->data[flat_idx * 32 + 7];
                *((uint8_t *)(&z) + 0) = pointcloud_shared->data[flat_idx * 32 + 8];
                *((uint8_t *)(&z) + 1) = pointcloud_shared->data[flat_idx * 32 + 9];
                *((uint8_t *)(&z) + 2) = pointcloud_shared->data[flat_idx * 32 + 10];
                *((uint8_t *)(&z) + 3) = pointcloud_shared->data[flat_idx * 32 + 11];
                //RCLCPP_INFO(this->get_logger(), "found %f %f %f", x, y, z);
                if(isfinite(x) && isfinite(y) && isfinite(z)) {
                    float pid_error = -x;
                    total_pid_error += pid_error;
                    twist.angular.z = pid_error * kp + total_pid_error * ki + (pid_error - last_pid_error) * kd;
                    last_pid_error = pid_error;
                    twist.linear.x = std::min(base_linear_speed, z);
                }
            }
            cmdvel_publisher->publish(twist);
            cv::waitKey(10);
        }
        
        rate.sleep();
    }
    
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sim2025FollowBall>();
    node->Run();
    rclcpp::shutdown();
    printf("hello world sim2025_follow_ball package\n");
    return 0;
}
