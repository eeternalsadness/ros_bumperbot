#ifndef TF_EXAMPLES_H
#define TF_EXAMPLES_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "bumperbot_examples/GetTransform.h"

class TfExamples {
public:
    TfExamples(const ros::NodeHandle &);

private:
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped static_transform_stamped_;
    geometry_msgs::TransformStamped dynamic_transform_stamped_;
    ros::Timer timer_;
    double last_x_;
    double x_increment_;
    ros::ServiceServer get_transform_srv_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void timerCallback(const ros::TimerEvent &);
    bool getTransformCallback(bumperbot_examples::GetTransform::Request &req,
                              bumperbot_examples::GetTransform::Response &res);
};

#endif