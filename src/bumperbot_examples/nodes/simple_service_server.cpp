// http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
// http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <ros/ros.h>
#include "bumperbot_examples/AddTwoInts.h"

bool addTwoInts(bumperbot_examples::AddTwoInts::Request &req,
                bumperbot_examples::AddTwoInts::Response &res){
    
    ROS_INFO_STREAM("Ready to sum " << req.a << " and " << req.b);
    res.sum = req.a + req.b;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_service_server_cpp");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("add_two_ints", addTwoInts);
    ROS_INFO("The service is ready to add ints");
    ros::spin();

    return 0;
}