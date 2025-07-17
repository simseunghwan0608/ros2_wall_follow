#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node {
public:
    WallFollow() : Node("wall_follow_node")
    {
        // Create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
           
    }

private:
    // PID CONTROL PARAMS
    double kp = 1.0;    // Proportional gain
    double kd = 0.1;    // Derivative gain  
    double ki = 0.01;   // Integral gain
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    
    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    
    // ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    

    
    double get_range(float* range_data, double angle, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg)
    {
        int index = (int)((angle - scan_msg->angle_min) / scan_msg->angle_increment);
        float range = range_data[index];

        return range ;
      
    }
    
    double get_error(float* range_data, double dist, const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg)
    {

        double angle_90 = -M_PI / 2.0;    
        double angle_45 = -M_PI / 4.0;     
        
        double range_90 = get_range(range_data, angle_90, scan_msg);
        double range_45 = get_range(range_data, angle_45, scan_msg);
        
        double alpha = atan((range_45 * cos(angle_45 - angle_90) - range_90) / range_45 * sin(angle_45- angle_90));
        double car_dis = range_90 * cos(alpha);
        double AC_term = 1.5 * sin(alpha);
        std:: cout << "차가 중심으로부터 회전각도 : "<< alpha <<"차가 벽으로부터 거리 : " << car_dis<< std::endl; 

        
        
        return  dist - (car_dis + AC_term ) ;
    
    }
    
    void pid_control(double error, double velocity)
    {
       
        double proportional = kp * error;
        integral += error;
        double integral_term = ki * integral;
        double derivative = kd * (error - prev_error);
        
        // Calculate steering angle
        double angle = proportional + integral_term + derivative + servo_offset;
        
        // Limit steering angle
        angle = std::max(-0.4, std::min(0.4, angle));
        
    
        // Create and publish drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        
        drive_publisher_->publish(drive_msg);
        
    
        prev_error = error;
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
       
        float* range_data = (float*)scan_msg->ranges.data();
        double desired_distance = 1.0; 
        
        double error = get_error(range_data, desired_distance, scan_msg);
        
        
        double base_velocity = 1.5;
        double velocity = base_velocity * (1.0 - std::min(std::abs(error), 1.0) * 0.5);
        velocity = std::max(0.5, velocity); 
        pid_control(error, velocity);
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
   
    
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}