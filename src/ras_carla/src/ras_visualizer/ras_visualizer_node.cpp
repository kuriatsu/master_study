#include <ros/ros.h>
#include "ras_visualizer.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ras_visualizer_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("ras_visualizer_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	RasVisualizer ras_visualizer;
	ROS_INFO("Ready...");
	// server->applyChanges();

	ros::spin();
	server.reset();

	return 0;
}
