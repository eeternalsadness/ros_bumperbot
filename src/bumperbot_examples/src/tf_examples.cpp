// http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html
// http://wiki.ros.org/tf2_ros?distro=noetic

#include "bumperbot_examples/tf_examples.h"
#include <tf2_ros/static_transform_broadcaster.h>

TfExamples::TfExamples(const ros::NodeHandle &nh) : nh_(nh) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    static_transform_stamped_.header.stamp = ros::Time::now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";

    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;

    static_transform_stamped_.transform.rotation.x = 0;
    static_transform_stamped_.transform.rotation.y = 0;
    static_transform_stamped_.transform.rotation.z = 0;
    static_transform_stamped_.transform.rotation.w = 1;

    static_broadcaster.sendTransform(static_transform_stamped_);
    ROS_INFO_STREAM("Publishing static transform between " << static_transform_stamped_.header.frame_id <<
                    " and " << static_transform_stamped_.child_frame_id);
}