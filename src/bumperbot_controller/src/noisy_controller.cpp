// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include "bumperbot_controller/noisy_controller.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <random>

NoisyController::NoisyController(const ros::NodeHandle &nh, double wheel_radius, double wheel_separation) 
    : nh_(nh), wheel_radius_(wheel_radius), wheel_separation_(wheel_separation), left_wheel_prev_pos_(0.0)
    , right_wheel_prev_pos_(0.0), x_(0.0), y_(0.0), theta_(0.0) {
    ROS_INFO_STREAM("Using wheel radius " << wheel_radius);
    ROS_INFO_STREAM("Using wheel separation " << wheel_separation);

    prev_time_ = ros::Time::now();

    // Odometry message
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.position.x = 0.0;
    odom_msg_.pose.pose.position.y = 0.0;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("bumperbot_controller/odom_noisy", 10);

    joint_sub_ = nh_.subscribe("joint_states", 1000, &NoisyController::jointCallback, this);

    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";
}

void NoisyController::jointCallback(const sensor_msgs::JointState &msg){
    // for(auto &n : msg.name){
    //     ROS_INFO_STREAM(n);
    // }
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0, 0.005);
    double wheel_encoder_left = msg.position.at(0) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = msg.position.at(1) + right_encoder_noise(noise_generator);

    double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
    double dp_right = wheel_encoder_right - right_wheel_prev_pos_;
    double dt = (msg.header.stamp - prev_time_).toSec();

    left_wheel_prev_pos_ = wheel_encoder_left;
    right_wheel_prev_pos_ = wheel_encoder_right;
    prev_time_ = msg.header.stamp;

    double phi_left = dp_left / dt;
    double phi_right = dp_right / dt;

    double linear = (wheel_radius_ * phi_right + wheel_radius_ * phi_left) / 2;
    double angular = (wheel_radius_ * phi_right - wheel_radius_ * phi_left) / wheel_separation_;

    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z =  angular;
    odom_msg_.header.stamp = ros::Time::now();

    odom_pub_.publish(odom_msg_);

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = ros::Time::now();

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transform_stamped_);
}