#include <ros/ros.h>

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
#include <tf/transform_listener.h>
#include "std_msgs/Int32.h"

// file read
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

class SwipeDetectorFixed
{
	private:
        // pub sub
        ros::Publisher pub_obstacle_pose;
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // ros::Publisher test_obstacle_pose;
        // ros::Publisher test_transform_origin;
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		ros::Subscriber sub_vehicle_pose;
        ros::Subscriber sub_waypoint_callback;

        // obstacle container setting
        const static int vector_size = 10;
		std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;

        // management information
        ros::Timer pub_timer;
        // int keep_time;
        // const static int appear_dist = 200;
        tf::TransformListener tf_listener;


        // ros::Time dis_timer_start;
		// bool dis_timer_flag;
        // const static int initial_waypoint = 0;

        // current robot information
        int round;
        geometry_msgs::Pose vehicle_pose;
        uint32_t next_waypoint_flag;

	public:
		SwipeDetectorFixed(const std::string &file_name);

	private:
		void readFile(const std::string &file_name);
		void pubTimerCallback(const ros::TimerEvent&);
        // void ndtPoseCallback(const geometry_msgs::PoseStamped &in_pose);
        void waypointCallback(const std_msgs::Int32 &in_msg);
        geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &in_pose, const std::string &current_frame_id, const std::string &target_frame_id);

};

SwipeDetectorFixed::SwipeDetectorFixed(const std::string &file_name): round(1)
{
    ros::NodeHandle n;
    // pub_erase_sinal = n.advertise<std_msgs::Int32>("/swipe_erase_signal", 5);
    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // test_transform_origin = n.advertise<geometry_msgs::PoseStamped>("/test_transform_origin", 5);
    // test_obstacle_pose = n.advertise<geometry_msgs::PoseStamped>("/test_obstacle_pose", 5);
// tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // sub_vehicle_pose = n.subscribe("/ndt_pose", 5, &SwipeDetectorFixed::ndtPoseCallback, this);
	sub_waypoint_callback = n.subscribe("/closest_waypoint", 5, &SwipeDetectorFixed::waypointCallback, this);

    // keep_time = 2.0;
    obstacle_vec.reserve(vector_size);
	readFile(file_name);

	ros::Duration(1).sleep();
	pub_timer = n.createTimer(ros::Duration(0.5), &SwipeDetectorFixed::pubTimerCallback, this);
}


// void SwipeDetectorFixed::ndtPoseCallback(const geometry_msgs::PoseStamped &in_pose)
// {
//     vehicle_pose = in_pose.pose;
// }


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
	swipe_obstacles::detected_obstacle read_obstacle;
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

		read_obstacle.pose.position.x = std::stof(result.at(0));
		read_obstacle.pose.position.y = std::stof(result.at(1));
		read_obstacle.pose.position.z = std::stof(result.at(2));
		read_obstacle.pose.orientation.x = std::stof(result.at(3));
		read_obstacle.pose.orientation.y = std::stof(result.at(4));
		read_obstacle.pose.orientation.z = std::stof(result.at(5));
        read_obstacle.pose.orientation.w = std::stof(result.at(6));
        read_obstacle.visible = std::stoi(result.at(7));
        read_obstacle.id = std::stoi(result.at(8));
        read_obstacle.shift_x = std::stof(result.at(9));
        read_obstacle.shift_y = std::stof(result.at(10));
        read_obstacle.score = 90.0;
        read_obstacle.label = "person";
        read_obstacle.header.frame_id = "map";

		obstacle_vec.push_back(read_obstacle);
		// ROS_INFO_STREAM(read_obstacle);
	}
}


void SwipeDetectorFixed::pubTimerCallback(const ros::TimerEvent&)
{
    swipe_obstacles::detected_obstacle_array out_array;
    geometry_msgs::Pose pose_from_velodyne;
    float distance;
    int flag=0;
    // std_msgs::Int32 erase_signal;
    // erase_signal.data = 0;

    for(auto i=obstacle_vec.begin(); i!=obstacle_vec.end(); i++)
    {
        if(i->visible == round)
        {
            pose_from_velodyne = tfTransformer(i->pose, i->header.frame_id, "/velodyne");

            // distance = std::pow(pose_from_velodyne.position.x, 2)+std::pow(pose_from_velodyne.position.y, 2);
            // std::cout << "distance:" << distance << std::endl;

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
                    i->detected_time = ros::Time::now();
                    std::cout << "published id:" << i->id << std::endl;

                    out_array.obstacles.push_back(*i);
                    out_array.header.frame_id = "map";
                    out_array.header.stamp = ros::Time::now();
                    flag = 1;
                }
            }
        }
    }
            // i->id ++;
            // if (i->only_at_once && !dis_timer_flag)
            // {
            //     dis_timer_flag = true;
            //     dis_timer_start = ros::Time::now();
            //     // i->detected_time = ros::Time::now();
            //
            // }else if(i->only_at_once && dis_timer_flag){
            //     // i->detected_time = ros::Time::now();
            //     if(i->detected_time - dis_timer_start > ros::Duration(10)){
            //         i->detected_time -= ros::Duration(3);
            //     }
            // }
            // ROS_INFO_STREAM(out_array.obstacles[0]);
    if(flag)
    {
        pub_obstacle_pose.publish(out_array);
        // last_pub_time = ros::Time::now();
    }
    // else
    // {
    //     if(ros::Time::now() - last_pub_time > ros::Duration(keep_time))
    //     {
    //         erase_signal.data = 1;
    //         pub_erase_sinal.publish(erase_signal);
    //     }
    // }
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

    // tf::Vector3 v = transform.getOrigin();
    // ROS_INFO_STREAM(v.x());

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
	// SwipeDetectorFixed swipe_detector_fixed("/home/kuriatsu/MAP/");

	ROS_INFO("detector ready...");

	ros::spin();
	return 0;
}
