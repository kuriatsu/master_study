#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

class SwipeDetectorYolo{

    private:
        ros::Publisher pub_obstacle_pose;
        ros::Subscriver sub_rect;

    public:
        SwipeDetectorYolo();

    private:
        void sub_rect_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msgs);
        void calc_3d_position(const autoware_msgs::DetectedObject::ConstPtr &rect)
};

SwipeDetectorYolo::SwipeDetectorYolo(){

    ros::NodeHandle n;

    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    sub_rect = n.subscrive("/detection/vision_objects", 5, &swipe_detector_yolo::sub_rect_callback, this)


}

void SwipeDetectorYolo::sub_rect_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msgs){

    for(size_t i=0; i<msgs->objects.size(); i++){
        calc_3d_position(msgs->objects[i]);
    }
}

void calc_3d_position(const autoware_msgs::DetectedObject::ConstPtr &rect){

    
}
