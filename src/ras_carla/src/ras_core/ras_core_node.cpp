#include <ros/ros.h>
#include "ras_core.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ras_core_node");

	ROS_INFO("initialized detector");
	RasCore ras_core;

	ros::spin();
	return 0;
}
