#ifndef TF_EXAMPLES_H
#define TF_EXAMPLES_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

class TfExamples {
public:
    TfExamples(const ros::NodeHandle &);

private:
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped static_transform_stamped_;
};

#endif