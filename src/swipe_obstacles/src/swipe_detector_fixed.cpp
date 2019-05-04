#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
#include <fstream>
// #include <string>
// #include <sstream>
// #include <iostream>

class swipe_detector_fixed{

	private:
		ros::Publisher pub_obstacle_pose;
		swipe_obstacles::detected_obstacle_array obstacle_list;
		ros::Timer timer;


	public:
		swipe_detector_fixed();

	private:
		void read_file();
		void pub_obstacle_pose_timer_callback(const ros::TimerEvent&);

};

swipe_detector_fixed::swipe_detector_fixed(){

	ros::NodeHandle n;

	pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);

	read_file();

	ros::Duration(1).sleep();
	timer = n.createTimer(ros::Duration(0.1), &swipe_detector_fixed::pub_obstacle_pose_timer_callback, this);

}

void swipe_detector_fixed::read_file(){

	swipe_obstacles::detected_obstacle read_obstacle;
	unsigned int id = 0;
	// std::cout << "read file" << std::endl;
	std::ifstream ifs("/home/kuriatsu/MAP/nu_garden/obstacle_pose.csv");
	if (!ifs){
		ROS_ERROR("Cannot Open File !");
		return;
	}
	std::string line;

	while(std::getline(ifs, line)){

		std::istringstream stream(line);
		std::string field;
		std::vector<float> result;

		while(std::getline(stream, field, ',')){
			result.push_back(std::stof(field));
		}

		read_obstacle.position.x = result.at(0);
		read_obstacle.position.y = result.at(1);
		read_obstacle.position.z = result.at(2);
		read_obstacle.orientation.x = result.at(3);
		read_obstacle.orientation.y = result.at(4);
		read_obstacle.orientation.z = result.at(5);
		read_obstacle.orientation.w = result.at(6);
		read_obstacle.id = id;

		obstacle_list.obstacles.push_back(read_obstacle);
		id++;
		// ROS_INFO_STREAM(read_obstacle);
	}
}

void swipe_detector_fixed::pub_obstacle_pose_timer_callback(const ros::TimerEvent&){
	ROS_INFO("published");
	pub_obstacle_pose.publish(obstacle_list);

}


int main(int argc, char **argv){

	ros::init(argc, argv, "swipe_detector_fixed_node");

	ROS_INFO("Initializing detector...");
	// ros::Duration(0.1).sleep();
	swipe_detector_fixed swipe_detector_fixed;
	ROS_INFO("detector ready...");

	ros::spin();
	return 0;
}
