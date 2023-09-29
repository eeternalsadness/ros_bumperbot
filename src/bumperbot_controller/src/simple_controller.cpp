#include "bumperbot_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>

SimpleController::SimpleController(const ros::NodeHandle &nh, double wheel_radius, double wheel_separation) : nh_(nh){
    ROS_INFO_STREAM("Using wheel radius " << wheel_radius);
    ROS_INFO_STREAM("Using wheel separation " << wheel_separation);

    right_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_controller/command", 10);
    left_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_controller/command", 10);

    vel_sub_ = nh_.subscribe("bumperbot_controller/cmd_vel", 1000, &SimpleController::velCallback, this);

    speed_conversion_ << wheel_radius / 2, wheel_radius / 2,
                         wheel_radius / wheel_separation, -wheel_radius / wheel_separation;
    
    ROS_INFO("The conversion matrix is \n%s", speed_conversion_);
}

void SimpleController::velCallback(const geometry_msgs::Twist& msg){
    Eigen::Vector2d robot_speed(msg.linear.x, msg.angular.z);
    
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;
    std_msgs::Float64 right_speed;
    right_speed.data = wheel_speed(0);
    std_msgs::Float64 left_speed;
    left_speed.data = wheel_speed(1);

    right_cmd_pub_.publish(right_speed);
    left_cmd_pub_.publish(left_speed);
}