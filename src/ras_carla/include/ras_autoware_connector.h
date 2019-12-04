#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include "std_msgs/Int32.h"

#include <cmath>


class ObstacleReflector{

private:
    ros::Publisher pub_pointcloud;
    ros::Subscriber sub_obstacles;
    ros::Subscriber sub_erase_signal;
    pcl::PointCloud<pcl::PointXYZRGB> in_cloud;
    ros::Timer timer;
    tf::TransformListener tf_listener;
    geometry_msgs::Twist ego_twist;
    geometry_msgs::Pose ego_pose;
    // ros::Time last_sub_time;
    // int keep_time;
    float polygon_interval;

    public :
    ObstacleReflector();
    void sync_jsk_box();
    void get_obstacle_pose();

    private :
    void make_cube();
    void eraseSignalCallback(const std_msgs::Int32 &in_msg);
    void subObstaclesCallback(const swipe_obstacles::detected_obstacle_array &in_msgs);
    void pubPointcloudNhz(const ros::TimerEvent&);
};
