#include <ros/ros.h>
#include "ras_detector_gt.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ras_detector_gt_node");

	ROS_INFO("initialized detector");
	DetectorGT ras_detector_gt;

	ros::spin();
	return 0;
}