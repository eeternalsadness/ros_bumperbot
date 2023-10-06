#include <ros/ros.h>
#include "bumperbot_examples/AddTwoInts.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_service_client_cpp");

    if(argc != 3){
        ROS_INFO("Requested two arguments");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<bumperbot_examples::AddTwoInts>("add_two_ints");

    bumperbot_examples::AddTwoInts srv;
    srv.request.a = atoi(argv[1]);
    srv.request.b = atoi(argv[2]);
    ROS_INFO_STREAM("Requesting " << srv.request.a << " + " << srv.request.b);

    if(client.call(srv)){
        ROS_INFO_STREAM("Service response " << srv.response.sum);
    }
    else{
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    
    return 0;
}