#include <ros/ros.h>
#include "ras_autoware_connector.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_reflector_node");
    ROS_INFO("Initializing...");
    ObstacleReflector obstacle_reflector;
    ROS_INFO("Ready...");
    ros::spin();

    return 0;
}
