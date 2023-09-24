#include "bumperbot_examples/turtlesim_kinematics.h"

TurtlesimKinematics::TurtlesimKinematics(){
    turtle1_pose_sub_ = nh_.subscribe("/turtle1/pose", 1000, &TurtlesimKinematics::turtle1PoseCallback, this);
    turtle2_pose_sub_ = nh_.subscribe("/turtle2/pose", 1000, &TurtlesimKinematics::turtle2PoseCallback, this);
}

void TurtlesimKinematics::turtle1PoseCallback(const turtlesim::Pose& pose){
    last_turtle1_pose_ = pose;
}

void TurtlesimKinematics::turtle2PoseCallback(const turtlesim::Pose& pose){
    last_turtle2_pose_ = pose;

    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;

    ROS_INFO_STREAM("----------------\n" <<
                    "Translation Vector turtle1 -> turtle2 \n" <<
                    "Tx: " << Tx << "\n" <<
                    "Ty: " << Ty << "\n");
}