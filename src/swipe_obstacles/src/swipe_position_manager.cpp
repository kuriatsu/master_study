#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

struct obstacle_info{

    uint32 id;
    float shift;
    geometry_msgs::Pose pose;

}

class PositionManager{

    private:
        ros::Publisher pub_obj;
        ros::Subscriber sub_obj;
        tf::TransformListener tf_listener;
        std_string origin_frame;

    public:
        PositionManager();

    private:
        sub_obj_callback(const swipe_obstacles::detected_obstacle_array &msgs);
        geometry_msgs::Pose tf_transformer(const geometry_msgs::Pose &in_pose);

};


PositionManager::PositionManager()
{
    ros::NodeHandle n;

    pub_obj = n.advertise<swipe_obstacles::detected_obstacle_array>("/managed_obstacles", 5);
    sub_obj = n.subscribe("/detected_obstacles", 5, &PositionManager::sub_obj_callback, this);
}


void SwipeDetectorYolo::sub_obj_callback(const  swipe_obstacles::detected_obstacle_array &msgs)
{
    swipe_obstacles::detected_obstacle_array out_obstacle_array;

    ROS_INFO("Get obj info");

    origin_frame = msg.header.frame_id;

    for(size_t i=0; i<msgs.objects.size(); i++)
    {
        out_obstacle_array.obstacles.push_back(calc_obj_info(msgs.objects[i]));
    }

}


geometry_msgs::Pose PositionManager::tf_transformer(const geometry_msgs::Pose &in_pose)
{
    geometry_msgs::Pose world_pose;
    tf::Pose world_to_camera;
    tf::Pose req_to_camera;
    tf::StampedTransform req_to_world;

    try{
		tf_listener.waitForTransform(origin_frame, "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", origin_frame, ros::Time(0), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}

	tf::poseMsgToTF(in_pose, world_to_camera);
	req_to_camera = req_to_world * world_to_camera;
	tf::poseTFToMsg(req_to_camera, world_pose);

    return world_pose;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_position_manager_node");

    ROS_INFO("Initialized");

    PositionManager position_manager;

    ros::spin();
    return 0;
}
// out_pose.pose = tf_transformer(camera_obj_pose);
