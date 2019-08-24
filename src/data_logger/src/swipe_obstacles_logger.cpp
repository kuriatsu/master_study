#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include "swipe_obstacles/detected_obstacle_array.h"
#include "data_logger/swipe_obstacles_log.h"

class SwipeObstaclesLogger
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_shift;
    ros::Subscriber sub_twist_raw;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_obstacle;
    ros::Publisher pub_log;

    data_logger::swipe_obstacles_log log;
    bool detected;
    float dist_to_pedestrian;

public:
    SwipeObstaclesLogger();

private:
    void joyCallback(const sensor_msgs::Joy &msg);
    void shiftCallback(const swipe_obstacles::detected_obstacle &msg);
    void twistRawCallback(const geometry_msgs::TwistStamped &msg);
    void odomCallback(const nav_msgs::Odometry &msg);
    void poseCallback(const geometry_msgs::Pose &msg);
    void obstacleCallback(const swipe_obstacles::detected_obstacle_array &msgs);
};


SwipeObstaclesLogger::SwipeObstaclesLogger()
{
    ros::NodeHandle n;

    sub_joy = n.subscribe("/joy", 1, &SwipeObstaclesLogger::joyCallback, this);
    sub_joy = n.subscribe("/shifted_info", 1, &SwipeObstaclesLogger::shiftCallback, this);
    sub_twist_raw = n.subscribe("/twist_raw", 1, &SwipeObstaclesLogger::TwistRawCallback, this);
    sub_odom = n.subscribe("/ypspur_ros/odom", 1, &SwipeObstaclesLogger::odomCallback, this);
    sub_pose = n.subscribe("/ndt_pose", 1, &SwipeObstaclesLogger::poseCallback, this);
    sub_obstacle = n.subscribe("/detected_obstacles", 1, &SwipeObstaclesLogger::obstacleCallback, this);

    pub_log = n.advertise("/data_log", 1);
}


void odomCallback(const nav_msgs::Odometry &msg)
{
    log.odom = msg.pose;
    log.ypspur_twist = msg.twist;

    log.header = ros::Time::now();
    pub_log.publish(log);
}


void joyCallback(const sensor_msgs::Joy &msg)
{
    log.accel = (1.0 - msg.axes[5]) * 0.5;
    log.brake = (1.0 - msg.axes[2]) * 0.5;
}


void shiftCallback(const swipe_obstacles::detected_obstacle &msg)
{
    log.shift = 1;
}

void twistRawCallback(const geometry_msgs::TwistStamped &msg)
{
    log.autoware_twist = msg.twist;
}


void poseCallback(const geometry_msgs::Pose &msg)
{
    log.pose = msg.pose;
}


void obstacleCallback(const swipe_obstacles::detected_obstacle_array &msgs)
{
    log.round = msgs
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_obstacles_log_node");
    SwipeObstaclesLogger swipe_obstacles_logger;
    ros::spin();
    return 0;
}
