#ifndef SIMPLE_CONTROLLER_H
#define SIMPLE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>

class SimpleController{
public:
    SimpleController(const ros::NodeHandle &, double wheel_radius, double wheel_separation);

private:
    void velCallback(const geometry_msgs::Twist &);
    void jointCallback(const sensor_msgs::JointState &);

    ros::NodeHandle nh_;
    ros::Publisher right_cmd_pub_;
    ros::Publisher left_cmd_pub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber joint_sub_;
    double wheel_radius_;
    double wheel_separation_;
    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    double x_;
    double y_;
    double theta_;
    ros::Time prev_time_;
    
    Eigen::Matrix2d speed_conversion_;
};

#endif