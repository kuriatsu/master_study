#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

class WaypointPublisher
{
private:
	ros::Publisher pub_waypoint;
	
	nav_msgs::Path path;

public:
	WaypointPublisher();

private:
	void readFile(const std::string &file_name);
};


WaypointPublisher::WaypointPublisher()
{
	ros::NodeHandle n;

	pub_waypoint = n.advertise<nav_msgs::Path>("/carla/ego_vehicle/waypoints", 1, true);

	std::string file_name;
	n.getParam("/file_name", file_name);
	std::cout << file_name << std::endl;
	readFile(file_name);

}


void WaypointPublisher::readFile(const std::string &file_name)
{
	std::string in_line;
	std::ifstream ifs(file_name);
	geometry_msgs::PoseStamped got_point;


	if(!ifs == 2)
	{
		ROS_ERROR("Cannot Open File!!!");
		return;
	}
	ROS_INFO("read file");

	std::getline(ifs, in_line);
	std::cout << in_line << std::endl;
	while (std::getline(ifs, in_line))
	{
		std::istringstream stream(in_line);
		std::string value;
		std::vector<std::string> got_line;

		while (std::getline(stream, value, ','))
		{
			got_line.emplace_back(value);
		}

		got_point.pose.position.x = std::stof(got_line.at(4));
		got_point.pose.position.y = std::stof(got_line.at(5));
		got_point.pose.position.z = std::stof(got_line.at(6));
		got_point.pose.orientation.x = std::stof(got_line.at(7));
		got_point.pose.orientation.y = std::stof(got_line.at(8));
		got_point.pose.orientation.z = std::stof(got_line.at(9));
		got_point.pose.orientation.w = std::stof(got_line.at(10));

		path.poses.push_back(got_point);
	}

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";
	ROS_INFO_STREAM(path);
	pub_waypoint.publish(path);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "carla_waypoint_publisher_node");

	ROS_INFO("initializing waypoint_publisher");

	WaypointPublisher waypoint_publisher;

	ros::spin();
	return 0;
}