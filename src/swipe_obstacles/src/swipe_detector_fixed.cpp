#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class detector_fixed_swipe{

	private:
		ros::Publisher pub_obstacle_pose;
		std::vector<tf::Vector3> obstacle_position;

	public:
		detector_fixed_swipe();

	private:
		void read_obstacle_position(const std::string *filename);
		void pub_obstacle_pose_timer_callback(const ros::TimerEvent&);

};

detector_fixed_swipe::detector_fixed_swipe(){

	ros::NodeHandle n;

	pub_obstacle_pose = n.advertise<std::vector<tf::Vector3>>("/detected_obstacles_swipe", 1);

	read_obstacle_position(filename);

	ros::Duration(1).sleep();
	timer = n.createTimer(ros::Duration(0.1), &detector_fixed_swipe::pub_obstacle_pose_timer_callback, this);

}

void detector_fixed_swipe::read_obstacle_position(const std::string *filename){

	geometry_msgs::Pose in_pose;

	std::cout << "read file" << std::endl;
	std::ifstream ifs(filename);
	std::string line;

	while(std::getline(ifs, line)){

		std::istringstream stream(line);
		std::string field;
		std::vector<float> result;

		while(std::getline(stream, field, ',')){
			result.push_back(std::stof(field));
		}

		in_pose.position.x = result.at(0);
		in_pose.position.y = result.at(1);
		in_pose.position.z = result.at(2);
		in_pose.orientation.x = result.at(3);
		in_pose.orientation.y = result.at(4);
		in_pose.orientation.z = result.at(5);
		in_pose.orientation.w = result.at(6);

		obstacle_pose.push_back(in_pose);
		ROS_INFO_STREAM(in_pose);
	}
}
