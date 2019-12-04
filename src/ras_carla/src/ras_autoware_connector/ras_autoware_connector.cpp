#include <ros/ros.h>
#include "ras_autoware_connector.h"

RasAutowareConnector::RasAutowareConnector()//: keep_time(2)
{
    ros::NodeHandle n;

    sub_obstacles = n.subscribe("/managed_obstacles", 5, &RasAutowareConnector::subObstaclesCallback, this);
}


void RasAutowareConnector::subObstaclesCallback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    double roll, pitch, yaw;
    double x, y;
    unsigned int count = 0;


    // std::cout << "sub" <<std::endl;
    in_cloud.points.resize(in_msgs.obstacles.size() * 20);
    in_cloud.header.frame_id = "world";

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

    // last_sub_time = ros::Time::now();
}
