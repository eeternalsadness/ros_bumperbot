#include "bumperbot_controller/simple_controller.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_controller_node");
    
    ros::NodeHandle pnh("~");
    double wheel_radius;
    pnh.getParam("~wheel_radius", wheel_radius);
    double wheel_separation;
    pnh.getParam("~wheel_separation", wheel_separation);

    ros::NodeHandle n;
    SimpleController controller(n, wheel_radius, wheel_separation);

    ros::spin();

    return 0;
}