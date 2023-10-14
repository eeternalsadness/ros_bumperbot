#ifndef NOISY_CONTROLLER_H
#define NOISY_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

class NoisyController{
public:
    NoisyController(const ros::NodeHandle &, double wheel_radius, double wheel_separation);

private:
    void jointCallback(const sensor_msgs::JointState &);

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber joint_sub_;
    double wheel_radius_;
    double wheel_separation_;
    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    double x_;
    double y_;
    double theta_;
    ros::Time prev_time_;
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped transform_stamped_;
};

#endif