#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>

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


    public:
        SwipeDetectorYolo(double in);

    private:
        void sub_obj_callback(const autoware_msgs::DetectedObjectArray &msgs);
        void sub_camera_info_callback(const sensor_msgs::CameraInfo &msgs);
        swipe_obstacles::detected_obstacle calc_obj_info(const autoware_msgs::DetectedObject &in_obj);
};


SwipeDetectorYolo::SwipeDetectorYolo(double in) : lidar_hight(in)
{
    ros::NodeHandle n;

    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    sub_obj = n.subscribe("/detection/vision_objects", 5, &SwipeDetectorYolo::sub_obj_callback, this);
    sub_camera_info = n.subscribe("/camera_info", 5, &SwipeDetectorYolo::sub_camera_info_callback, this);
}


// suscribe object positions array in 2D image
// publish objects array in 3D world. frame_id is same as input obstacles.
void SwipeDetectorYolo::sub_obj_callback(const autoware_msgs::DetectedObjectArray &msgs)
{
    swipe_obstacles::detected_obstacle_array out_obstacle_array;

    // calc each obstacle position and put them into container
    for(size_t i=0; i<msgs.objects.size(); i++)
    {
        out_obstacle_array.obstacles.push_back(calc_obj_info(msgs.objects[i]));
    }

    // add adititonal informations
    out_obstacle_array.header.frame_id = camera_frame_id;
    out_obstacle_array.header.stamp = ros::Time::now();

    // publish
    pub_obstacle_pose.publish(out_obstacle_array);
}


// input: yolo rectangle position in 2D image
// output: 3D position
swipe_obstacles::detected_obstacle SwipeDetectorYolo::calc_obj_info(const autoware_msgs::DetectedObject &in_obj)
{
    swipe_obstacles::detected_obstacle out_pose;

    // calc obstacle position using camera intrinsic parametors
    out_pose.pose.position.y = lidar_hight;
    out_pose.pose.position.z = lidar_hight * camera_mat[4] / ((in_obj.y + in_obj.height) - camera_mat[5]);
    out_pose.pose.position.x = (((in_obj.x + in_obj.width / 2) - camera_mat[2]) * out_pose.pose.position.z) / camera_mat[0];

    // for data sampling
    //std::cout << out_pose.pose.position.z << "," << out_pose.pose.position.x << std::endl;

    // add adtional informations
    out_pose.id = in_obj.id;
    out_pose.label = in_obj.label;
    out_pose.score = in_obj.score;

    return out_pose;
}


// input: camera infor
// save informations to class valiables
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
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_detector_yolo_node");

    ROS_INFO("Initialized");
    SwipeDetectorYolo swipe_detector_yolo(0.96);

    ros::spin();
    return 0;
}
