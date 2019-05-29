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
        ros::Subscriver sub_rect;
        ros::Subscriver sub_camera_info;
        double lidar_hight;
        double camera_mat[9];

    public:
        SwipeDetectorYolo(double in):lidar_hight(in);

    private:
        void sub_rect_callback(const autoware_msgs::DetectedObjectArray &msgs);
        void sub_camera_info_callback(const sensor_msgs::CameraInfo &msgs);
        swipe_obstacles::detected_obstacle calc_3d_position(const autoware_msgs::DetectedObject &in_rect)
};

SwipeDetectorYolo::SwipeDetectorYolo(double in) : lidar_hight(in){

    ros::NodeHandle n;

    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    sub_rect = n.subscrive("/detection/vision_objects", 5, &swipe_detector_yolo::sub_rect_callback, this)
    sub_rect = n.subscrive("/camera_info", 5, &swipe_detector_yolo::sub_camera_info_callback, this)


}

void SwipeDetectorYolo::sub_rect_callback(const autoware_msgs::DetectedObjectArray &msgs){

    swipe_obstacles::detected_obstacle_array out_obstacle_array;

    for(size_t i=0; i<msgs.objects.size(); i++){
        out_obstacle_array.push_back(calc_3d_position(msgs.objects[i], out_obstacle_array));
    }

    pub_obstacle_pose(out_obstacle_array);
}

swipe_obstacles::detected_obstacle SwipeDetectorYolo::calc_3d_position(const autoware_msgs::DetectedObject &in_rect){

    swipe_obstacles::detected_obstacle out_pos;
    // camera_mat = cv::Mat(3, 3, )

    out_pos.position.y = lidar_hight;
    out_pos.position.z = out_pos.position.y * camera_mat[4] / (in_rect.y - camera_mat[5]);
    out_pos.position.x = ((in_rect.x - camera_mat[2]) * out_pos.position.z) / camera_mat[0];

    return out_pos;
}

void SwipeDetectorYolo::sub_camera_info_callback(const sensor_msgs::CameraInfo &msgs){

    camera_mat = msgs.K;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "swipe_detector_yolo_node");

    SwipeDetectorYolo swipe_detector_yolo(1.2);

    ros::spin();
    return 0;
}
