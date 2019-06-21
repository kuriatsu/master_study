#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

#include "std_msgs/Int32.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

class SwipeDetectorFixed{

	private:
        ros::Publisher pub_obstacle_pose;
        ros::Publisher pub_erase_sinal;
		ros::Subscriber sub_vehicle_pose;
        ros::Subscriber sub_waypoint_callback;

		std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;
		ros::Timer timer;
        ros::Time last_pub_time;
        const static int vector_size = 10;
        const static int appear_dist = 50;
        const static int initial_waypoint = 0;
        int round;
        int keep_time;
        geometry_msgs::Pose vehicle_pose;

	public:
		SwipeDetectorFixed();

	private:
		void read_file();
		void pub_obstacle_pose_timer_callback(const ros::TimerEvent&);
        void ndt_pose_callback(const geometry_msgs::PoseStamped &in_pose);
        void waypoint_callback(const std_msgs::Int32 &in_msg);
};

SwipeDetectorFixed::SwipeDetectorFixed(): round(1),  keep_time(2.0)
{
    ros::NodeHandle n;
    pub_erase_sinal = n.advertise<std_msgs::Int32>("/swipe_erase_signal", 5);
    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    sub_vehicle_pose = n.subscribe("/ndt_pose", 5, &SwipeDetectorFixed::ndt_pose_callback, this);
	sub_waypoint_callback = n.subscribe("/closest_waypoint", 5, &SwipeDetectorFixed::waypoint_callback, this);

    obstacle_vec.reserve(vector_size);
	read_file();

	ros::Duration(1).sleep();
	timer = n.createTimer(ros::Duration(0.5), &SwipeDetectorFixed::pub_obstacle_pose_timer_callback, this);
}


void SwipeDetectorFixed::ndt_pose_callback(const geometry_msgs::PoseStamped &in_pose)
{
    vehicle_pose = in_pose.pose;
}


void SwipeDetectorFixed::waypoint_callback(const std_msgs::Int32 &in_msg)
{
    if (in_msg.data == 0)
    {
        round += 1;
    }
}


void SwipeDetectorFixed::read_file(){

	swipe_obstacles::detected_obstacle read_obstacle;
	// std::cout << "read file" << std::endl;
	std::ifstream ifs("/home/kuriatsu/MAP/nu_garden/obstacle_pose_circle.csv");
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
            // std::cout << field << std::endl;
			result.push_back(std::stof(field));
		}

		read_obstacle.pose.position.x = result.at(0);
		read_obstacle.pose.position.y = result.at(1);
		read_obstacle.pose.position.z = result.at(2);
		read_obstacle.pose.orientation.x = result.at(3);
		read_obstacle.pose.orientation.y = result.at(4);
		read_obstacle.pose.orientation.z = result.at(5);
        read_obstacle.pose.orientation.w = result.at(6);
        read_obstacle.visible = result.at(7);
        read_obstacle.id = result.at(8);
        read_obstacle.shift_x = result.at(9);
        read_obstacle.shift_y = result.at(10);
        read_obstacle.score = 90.0;
        read_obstacle.label = "person";
        read_obstacle.header.frame_id = "map";

		obstacle_vec.push_back(read_obstacle);
		// ROS_INFO_STREAM(read_obstacle);
	}
}

void SwipeDetectorFixed::pub_obstacle_pose_timer_callback(const ros::TimerEvent&)
{
    swipe_obstacles::detected_obstacle_array out_array;
    float distance;
    int flag=0;
    std_msgs::Int32 erase_signal;
    // erase_signal.data = 0;

    for(auto i=obstacle_vec.begin(); i!=obstacle_vec.end(); i++)
    {
        distance = std::pow(i->pose.position.x-vehicle_pose.position.x, 2)+std::pow(i->pose.position.y-vehicle_pose.position.y, 2);
        if(distance < float(appear_dist) && i->visible == round)
        {
            // i->id ++;
            i->detected_time = ros::Time::now();
            out_array.obstacles.push_back(*i);
            out_array.header.frame_id = "map";
            std::cout << "published id:" << i->id << std::endl;
            flag = 1;
            ROS_INFO_STREAM(out_array.obstacles[0]);
        }
    }
    if(flag)
    {
        pub_obstacle_pose.publish(out_array);
        last_pub_time = ros::Time::now();
    }else
    {
        if(ros::Time::now() - last_pub_time > ros::Duration(keep_time))
        {
            erase_signal.data = 1;
            pub_erase_sinal.publish(erase_signal);
        }
    }
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
