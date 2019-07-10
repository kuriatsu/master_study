#include <ros/ros.h>

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
#include "swipe_obstacles/closest_obstacle.h"
#include <tf/transform_listener.h>
#include "std_msgs/Int32.h"

// file read
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

struct read_obstacle
{
    swipe_obstacles::detected_obstacle detected_obstacle;
    swipe_obstacles::closest_obstacle closest_obstacle;
};

class SwipeDetectorFixed
{
	private:
        // pub sub
        ros::Publisher pub_obstacle_pose;
        ros::Publisher pub_closest_obstacle;
        ros::Publisher pub_erase_signal;
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // ros::Publisher test_obstacle_pose;
        // ros::Publisher test_transform_origin;
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		ros::Subscriber sub_vehicle_pose;
        ros::Subscriber sub_waypoint_callback;

        // obstacle container setting
        const static int vector_size = 20;
		std::vector<read_obstacle> read_obstacle_vec;

        // management information
        ros::Timer pub_timer;
        int keep_time;
        ros::Time last_pub_time;
        tf::TransformListener tf_listener;

        int round;
        geometry_msgs::Pose vehicle_pose;
        uint32_t next_waypoint_flag;

	public:
		SwipeDetectorFixed(const std::string &file_name);

	private:
		void readFile(const std::string &file_name);
		void pubTimerCallback(const ros::TimerEvent&);
        void waypointCallback(const std_msgs::Int32 &in_msg);
        geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &in_pose, const std::string &current_frame_id, const std::string &target_frame_id);

};


SwipeDetectorFixed::SwipeDetectorFixed(const std::string &file_name): round(1)
{
    ros::NodeHandle n;
    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    pub_closest_obstacle = n.advertise<swipe_obstacles::closest_obstacle>("/closest_obstacle", 5);
    pub_erase_signal = n.advertise<std_msgs::Int32>("/swipe_erase_signal", 5);
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // test_transform_origin = n.advertise<geometry_msgs::PoseStamped>("/test_transform_origin", 5);
    // test_obstacle_pose = n.advertise<geometry_msgs::PoseStamped>("/test_obstacle_pose", 5);
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	sub_waypoint_callback = n.subscribe("/closest_waypoint", 5, &SwipeDetectorFixed::waypointCallback, this);

    read_obstacle_vec.reserve(vector_size);
	readFile(file_name);

	ros::Duration(1).sleep();
	pub_timer = n.createTimer(ros::Duration(0.5), &SwipeDetectorFixed::pubTimerCallback, this);
}


void SwipeDetectorFixed::waypointCallback(const std_msgs::Int32 &in_msg)
{
    if (in_msg.data == 0)
    {
        next_waypoint_flag = 1;
    }
    else if(in_msg.data == 1)
    {
        next_waypoint_flag = 2;
    }
    else if(in_msg.data == next_waypoint_flag)
    {
        next_waypoint_flag = 0;
        round++;
        ROS_INFO_STREAM(round);
    }
}


void SwipeDetectorFixed::readFile(const std::string &file_name)
{
	read_obstacle read_obstacle;
    std::string line;
    std::ifstream ifs(file_name);

	if (!ifs == 2)
    {
		ROS_ERROR("Cannot Open File !");
		return;
	}

    // skip first line
    std::getline(ifs, line);
    //read csv file
	while(std::getline(ifs, line))
    {
		std::istringstream stream(line);
		std::string value;
		std::vector<std::string> result;

		while(std::getline(stream, value, ','))
        {
			result.push_back(value);
		}

        read_obstacle.detected_obstacle.id = std::stoi(result.at(0));
        read_obstacle.detected_obstacle.round = std::stoi(result.at(1));
		read_obstacle.detected_obstacle.pose.position.x = std::stof(result.at(2));
		read_obstacle.detected_obstacle.pose.position.y = std::stof(result.at(3));
		read_obstacle.detected_obstacle.pose.position.z = std::stof(result.at(4));
		read_obstacle.detected_obstacle.pose.orientation.x = std::stof(result.at(5));
		read_obstacle.detected_obstacle.pose.orientation.y = std::stof(result.at(6));
		read_obstacle.detected_obstacle.pose.orientation.z = std::stof(result.at(7));
        read_obstacle.detected_obstacle.pose.orientation.w = std::stof(result.at(8));
        read_obstacle.detected_obstacle.shift_x = std::stof(result.at(9));
        read_obstacle.detected_obstacle.shift_y = std::stof(result.at(10));
        read_obstacle.detected_obstacle.label = result.at(11);
        read_obstacle.detected_obstacle.score = std::stof(result.at(12));
        read_obstacle.detected_obstacle.header.frame_id = result.at(13);

        read_obstacle.closest_obstacle.id = std::stoi(result.at(0));
        read_obstacle.closest_obstacle.brief_stop = std::stoi(result.at(14));
        read_obstacle.closest_obstacle.stop_time = std::stof(result.at(15));

        read_obstacle_vec.push_back(read_obstacle);
		// ROS_INFO_STREAM(read_obstacle);
	}
}


void SwipeDetectorFixed::pubTimerCallback(const ros::TimerEvent&)
{
    swipe_obstacles::detected_obstacle_array out_array;
    swipe_obstacles::closest_obstacle closest_obstacle;
    geometry_msgs::Pose pose_from_velodyne;
    float closest_obstacle_distance = 10.0;
    int pub_flag=0;
    std_msgs::Int32 erase_signal;
    // erase_signal.data = 0;

    for(auto i=read_obstacle_vec.begin(); i!=read_obstacle_vec.end(); i++)
    {
        if(i->detected_obstacle.round == round)
        {
            pose_from_velodyne = tfTransformer(i->detected_obstacle.pose, i->detected_obstacle.header.frame_id, "/velodyne");

// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            // std::cout << i->header.frame_id << "to" << "velodyne" << std::endl;
            // ROS_INFO_STREAM(i->pose);
            // ROS_INFO("obstacle on velodyne");
            // ROS_INFO_STREAM(pose_from_velodyne);
            // geometry_msgs::PoseStamped pose_from_velodyne_stamp;
            // pose_from_velodyne_stamp.header.stamp = ros::Time::now();
            // pose_from_velodyne_stamp.header.frame_id = "velodyne";
            // pose_from_velodyne_stamp.pose = pose_from_velodyne;
            // test_obstacle_pose.publish(pose_from_velodyne_stamp);
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if(0.0 < pose_from_velodyne.position.x && pose_from_velodyne.position.x < 10.0)
            {
                if(-5.0 < pose_from_velodyne.position.y && pose_from_velodyne.position.y < 5.0)
                {
                    // add obstacle to array
                    i->detected_obstacle.detected_time = ros::Time::now();
                    std::cout << "published id:" << i->detected_obstacle.id << std::endl;
                    out_array.obstacles.push_back(i->detected_obstacle);
                    // find closest obstacle
                    if(pose_from_velodyne.position.x < closest_obstacle_distance)
                    {
                        closest_obstacle = i->closest_obstacle;
                        closest_obstacle.distance = pose_from_velodyne.position.x;
                    }
                    pub_flag = 1;
                }
            }
        }
    }

    if(pub_flag)
    {
        // publish array
        out_array.header.frame_id = "map";
        out_array.header.stamp = ros::Time::now();
        pub_obstacle_pose.publish(out_array);
        //publish closest obstacle
        closest_obstacle.header.frame_id = "velodyne";
        closest_obstacle.header.stamp = ros::Time::now();
        pub_closest_obstacle.publish(closest_obstacle);
        // ROS_INFO_STREAM(closest_obstacle);
        last_pub_time = ros::Time::now();
    }
    else
    {
        if(ros::Time::now() - last_pub_time > ros::Duration(keep_time))
        {
            erase_signal.data = 1;
            pub_erase_signal.publish(erase_signal);
        }
    }
}


geometry_msgs::Pose SwipeDetectorFixed::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
    tf::Pose current_tf;
    tf::StampedTransform transform;
    tf::Pose transformed_tf;
    geometry_msgs::Pose transformed_pose;
    geometry_msgs::PoseStamped transform_origin;

    try{
		tf_listener.waitForTransform(current_frame_id, target_frame_id,  ros::Time(0), ros::Duration(1.0));
        // current_frame_id　から　target_frame_id　への座標変換
		tf_listener.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), transform);

	}catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
    }


// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // // getorigin は変換後の座標における変換前の座標の原点frame_id=target_frame_idにするとgetoriginは変換前のフレームの原点にoriginがあることがわかる
    // transform_origin.header.stamp = ros::Time::now();
    // transform_origin.header.frame_id = "velodyne";
    // transform_origin.pose.position.x = transform.getOrigin().x();
    // transform_origin.pose.position.y = transform.getOrigin().y();
    // transform_origin.pose.position.z = transform.getOrigin().z();
    // transform_origin.pose.orientation.x = transform.getRotation().x();
    // transform_origin.pose.orientation.y = transform.getRotation().y();
    // transform_origin.pose.orientation.z = transform.getRotation().z();
    // transform_origin.pose.orientation.w = transform.getRotation().w();
    // test_transform_origin.publish(transform_origin);
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	tf::poseMsgToTF(current_pose, current_tf);
	transformed_tf = transform * current_tf;
	tf::poseTFToMsg(transformed_tf, transformed_pose);

    return transformed_pose;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "swipe_detector_fixed_node");

	ROS_INFO("Initializing detector...");
	// ros::Duration(0.1).sleep();
    SwipeDetectorFixed swipe_detector_fixed("/home/kuriatsu/MAP/nu_garden/obstacle_pose_circle.csv");
	// SwipeDetectorFixed swipe_detector_fixed("/home/kuriatsu/MAP/takeda_lab/obstacle_takeda_lab.csv");

	ROS_INFO("detector ready...");

	ros::spin();
	return 0;
}
