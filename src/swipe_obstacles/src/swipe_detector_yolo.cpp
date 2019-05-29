#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

// #include <opencv2/opencv.hpp>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

class SwipeDetectorYolo{

    private:
        ros::Publisher pub_obstacle_pose;
        ros::Subscriber sub_obj;
        ros::Subscriber sub_camera_info;
        double lidar_hight;
        double camera_mat[9] = {0.0};
        std::string camera_frame_id;
        tf::TransformListener tf_listener;


    public:
        SwipeDetectorYolo(double in);

    private:
        void sub_obj_callback(const autoware_msgs::DetectedObjectArray &msgs);
        void sub_camera_info_callback(const sensor_msgs::CameraInfo &msgs);
        swipe_obstacles::detected_obstacle calc_obj_info(const autoware_msgs::DetectedObject &in_obj);
        geometry_msgs::Pose tf_transformer(const geometry_msgs::Pose &in_pose);
};

SwipeDetectorYolo::SwipeDetectorYolo(double in) : lidar_hight(in){

    ros::NodeHandle n;

    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    sub_obj = n.subscribe("/detection/vision_objects", 5, &SwipeDetectorYolo::sub_obj_callback, this);
    sub_camera_info = n.subscribe("/camera_info", 5, &SwipeDetectorYolo::sub_camera_info_callback, this);


}

void SwipeDetectorYolo::sub_obj_callback(const autoware_msgs::DetectedObjectArray &msgs){

    swipe_obstacles::detected_obstacle_array out_obstacle_array;
    ROS_INFO("Get obj info");

    for(size_t i=0; i<msgs.objects.size(); i++){
        out_obstacle_array.obstacles.push_back(calc_obj_info(msgs.objects[i]));
    }

    out_obstacle_array.header.frame_id = "world";
    out_obstacle_array.header.stamp = ros::Time::now();
    pub_obstacle_pose.publish(out_obstacle_array);
}


swipe_obstacles::detected_obstacle SwipeDetectorYolo::calc_obj_info(const autoware_msgs::DetectedObject &in_obj){

    swipe_obstacles::detected_obstacle out_pose;
    geometry_msgs::Pose camera_obj_pose;

    // camera_mat = cv::Mat(3, 3, )

    camera_obj_pose.position.y = lidar_hight;
    camera_obj_pose.position.z = lidar_hight * camera_mat[4] / ((in_obj.y + in_obj.height) - camera_mat[5]);
    camera_obj_pose.position.x = (((in_obj.x + in_obj.width / 2) - camera_mat[2]) * camera_obj_pose.position.z) / camera_mat[0];

    out_pose.pose = tf_transformer(camera_obj_pose);
    out_pose.id = in_obj.id;
    out_pose.label = in_obj.label;
    out_pose.score = in_obj.score;

    return out_pose;
}


geometry_msgs::Pose SwipeDetectorYolo::tf_transformer(const geometry_msgs::Pose &in_pose)
{
    geometry_msgs::Pose world_pose;
    tf::Pose world_to_camera;
    tf::Pose req_to_camera;
    tf::StampedTransform req_to_world;

    try{
		tf_listener.waitForTransform(camera_frame_id, "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", camera_frame_id, ros::Time(0), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}

	tf::poseMsgToTF(in_pose, world_to_camera);
	req_to_camera = req_to_world * world_to_camera;
	tf::poseTFToMsg(req_to_camera, world_pose);

    return world_pose;
}

void SwipeDetectorYolo::sub_camera_info_callback(const sensor_msgs::CameraInfo &msgs)
{
    if(camera_mat[0]==0.0)
    {
        ROS_INFO("Get camera info");

        camera_frame_id = msgs.header.frame_id;
        for (int i=0;i<9;i++)
        {
            camera_mat[i] = msgs.K[i];
        }
        ROS_INFO_STREAM(camera_frame_id);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "swipe_detector_yolo_node");

    ROS_INFO("Initialized");
    SwipeDetectorYolo swipe_detector_yolo(0.96);

    ros::spin();
    return 0;
}
