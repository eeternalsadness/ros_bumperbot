#include "bumperbot_controller/noisy_controller.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "noisy_controller");

    // need a node handle with a private namespace to access the private parameters
    ros::NodeHandle pnh("~");
    double wheel_radius;
    pnh.getParam("wheel_radius", wheel_radius);
    double wheel_separation;
    pnh.getParam("wheel_separation", wheel_separation);

    ros::NodeHandle nh;
    NoisyController controller(nh, wheel_radius, wheel_separation);

    ros::spin();

    return 0;
}