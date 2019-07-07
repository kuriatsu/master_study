#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
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
    sensor_msgs::PointCloud2 in_scan;
    ros::Timer timer;
    tf::TransformListener tf_listener;
    ros::Time last_sub_time;
    int keep_time;

    public :
    ObstacleReflector();
    void sync_jsk_box();
    void get_obstacle_pose();

    private :
    void make_cube();
    // void erase_signal_callback(const std_msgs::Int32 &in_msg);
    void sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
    void pub_pointcloud_Nhz(const ros::TimerEvent&);
};


ObstacleReflector::ObstacleReflector(): keep_time(2)
{
    ros::NodeHandle n;

    sub_obstacles = n.subscribe("/managed_obstacles", 5, &ObstacleReflector::sub_obstacles_callback, this);
    sub_erase_signal = n.subscribe("/swipe_erase_signal", 1, &ObstacleReflector::erase_signal_callback, this);
    // pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/points_raw", 1);
    pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/int_pointcloud", 1);

    // in_cloud.clear();
    // in_cloud.width = 20;
    // in_cloud.height = 5;

    ros::Duration(0.5).sleep();
    timer = n.createTimer(ros::Duration(0.2), &ObstacleReflector::pub_pointcloud_Nhz, this);
}


void ObstacleReflector::erase_signal_callback(const std_msgs::Int32 &in_msg)
{
    if(in_msg.data)
    {
        in_scan.clear();
        in_scan.header.stamp = ros::Time::now();
        in_scan.header.frame_id = "velodyne";
        pub_pointcloud.publish(in_scan);
    }
}


void ObstacleReflector::sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    pcl::PointCloud<pcl::PointXYZRGB> in_cloud;
    double roll, pitch, yaw;
    double x, y;
    unsigned int count = 0;


    // std::cout << "sub" <<std::endl;
    in_cloud.points.resize(in_msgs.obstacles.size() * 20);

    for (size_t i=0; i < in_msgs.obstacles.size(); i++)
    {
        tf::Quaternion tf_quat(in_msgs.obstacles[i].pose.orientation.x, in_msgs.obstacles[i].pose.orientation.y, in_msgs.obstacles[i].pose.orientation.z, in_msgs.obstacles[i].pose.orientation.w);
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        for (unsigned int col = 0; col < 20; col++)
        {
            pcl::PointXYZRGB &point = in_cloud.points[count];
            y = -3.0 + col * 0.3;
            point.x = - y*sin(2*M_PI+yaw) + in_msgs.obstacles[i].pose.position.x + in_msgs.obstacles[i].shift_x;
            point.y = y*cos(2*M_PI+yaw) + in_msgs.obstacles[i].pose.position.y + in_msgs.obstacles[i].shift_y;
            point.z = in_msgs.obstacles[i].pose.position.z;
            point.r = point.g = point.b = 0.3;
            count++;
        }
    }

    pcl::toROSMsg(in_cloud, in_scan);
    in_scan.header.frame_id = "world";

    last_sub_time = ros::Time::now();
}


void ObstacleReflector::pub_pointcloud_Nhz(const ros::TimerEvent&)
{
    sensor_msgs::PointCloud2 out_scan;
    // out_scan.header.frame_id = "world";

    if(!in_cloud.empty())
    {
        try{
            tf_listener.waitForTransform("world", "velodyne", ros::Time(0), ros::Duration(0.2));
        }catch (tf::TransformException &ex)  {
            ROS_ERROR("%s", ex.what());
        }

        pcl_ros::transformPointCloud("velodyne", in_scan, out_scan, tf_listener);
        std::cout << "points published" << std::endl;
        out_scan.header.frame_id = "velodyne";
        out_scan.header.stamp = ros::Time::now();
        pub_pointcloud.publish(out_scan);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_reflector_node");
    ROS_INFO("Initializing...");
    ObstacleReflector obstacle_reflector;
    ROS_INFO("Ready...");
    ros::spin();

    return 0;
}
