#include "bumperbot_localization/kalman_filter.h"

KalmanFilter::KalmanFilter(const ros::NodeHandle &nh) 
    : nh_(nh), mean_(0.0), variance_(1000.0), imu_angular_z_(0.0), is_first_odom_(true)
    , last_angular_z_(0.0), motion_(0.0) {
    odom_sub_ = nh_.subscribe("bumperbot_controller/odom_noisy", 1000, &KalmanFilter::odomCallback, this);
    imu_sub_ = nh_.subscribe("imu", 1000, &KalmanFilter::imuCallback, this);
    
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("bumperbot_controller/odom_kalman", 10);
}

void KalmanFilter::imuCallback(const sensor_msgs::Imu &imu){
    imu_angular_z_ = imu.angular_velocity.z;
}

void KalmanFilter::odomCallback(const nav_msgs::Odometry &odom){
    kalman_odom_ = odom;

    if(is_first_odom_){
        mean_ = odom.twist.twist.angular.z;
        last_angular_z_ = odom.twist.twist.angular.z;
        is_first_odom_ = false;
        return;
    }

    statePrediction();
    measurementUpdate();
}