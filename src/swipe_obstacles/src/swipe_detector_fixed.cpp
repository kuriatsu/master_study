#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

#include <fstream>
// #include <string>
// #include <sstream>
// #include <iostream>

class SwipeDetectorFixed{

	private:
        ros::Publisher pub_obstacle_pose;
		ros::Subscriber sub_vehicle_pose;

		std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;
		ros::Timer timer;
        const static int vector_size = 10;
        const static int appear_dist = 50;
        int round;
        geometry_msgs::Pose vehicle_pose;

	public:
		SwipeDetectorFixed();

	private:
		void read_file();
		void pub_obstacle_pose_timer_callback(const ros::TimerEvent&);
        void sub_vehicle_pose_callback(const tf::)
};

SwipeDetectorFixed::SwipeDetectorFixed()
{
	ros::NodeHandle n;
    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
	sub_vehicle_pose = n.subscribe("/pose", 5, &SwipeDetectorFixed::sub_vehicle_pose_callback, this);
    round = 0;

    obstacle_vec.reserve(vector_size);
	read_file();

	ros::Duration(1).sleep();
	timer = n.createTimer(ros::Duration(0.1), &SwipeDetectorFixed::pub_obstacle_pose_timer_callback, this);
}


void sub_vehicle_pose_callback(const geometry_msgs::Pose &in_pose)
{
    vehicle_pose = in_pose;
}


void SwipeDetectorFixed::read_file(){

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

		read_obstacle.pose.position.x = result.at(0);
		read_obstacle.pose.position.y = result.at(1);
		read_obstacle.pose.position.z = result.at(2);
		read_obstacle.pose.orientation.x = result.at(3);
		read_obstacle.pose.orientation.y = result.at(4);
		read_obstacle.pose.orientation.z = result.at(5);
		read_obstacle.pose.orientation.w = result.at(6);
        read_obstacle.id = id;
        read_obstacle.score = 90.0;
        read_obstacle.label = "person";

		obstacle_vec.push_back(read_obstacle);
		id++;
		// ROS_INFO_STREAM(read_obstacle);
	}
}

void SwipeDetectorFixed::pub_obstacle_pose_timer_callback(const ros::TimerEvent&)
{
    swipe_obstacles::detected_obstacle_array out_array;
    float distance;

    for(auto i=obstacle_vec.begin(); i!=obstacle_vec.end(); i++)
    {
        distance = (i->pose.position.x-vehicle_pose.position.x)**2+(i->pose.position.y-vehicle_pose.position.y)**2;
        if(distance < float(appear_dist))
        {
            i->id += round;
            i->detected_time = ros::Time(0);
            out_array.push_back(*i);
        }
    }
    pub_obstacle_pose.publish(out_array);
    ROS_INFO("published");

}


int main(int argc, char **argv){

	ros::init(argc, argv, "swipe_detector_fixed_node");

	ROS_INFO("Initializing detector...");
	// ros::Duration(0.1).sleep();
	SwipeDetectorFixed swipe_detector_fixed;
	ROS_INFO("detector ready...");

	ros::spin();
	return 0;
}
