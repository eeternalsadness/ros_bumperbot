#include <ros/ros.h>
#include <std_msgs/String.h>

void msgCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("New message received: %s", msg -> data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_cpp_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 10, msgCallback);
    ros::spin();

    return 0;
}