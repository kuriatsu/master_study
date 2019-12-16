#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <math.h>
#include <unordered_map>
#include <nav_msgs/Odometry.h>

#include "ras_lib.h"
#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"

#include "carla_msgs/CarlaActorList.h"

class RasCore
{
private:
    ros::Publisher pub_obj;
	ros::Subscriber sub_carla_actor_list;
	ros::Subscriber sub_carla_obj;
    ros::Subscriber sub_shift;
	ros::Subscriber sub_odom;

    int keep_time;
    float max_recognize_distance;
    float min_recognize_distance;
	float min_recognize_vel;
    std::unordered_map<int, ras_carla::RasObject> obj_map;
    float polygon_interval;
    geometry_msgs::Pose ego_pose;
    geometry_msgs::Twist ego_twist;
    int ego_id;

public:
	RasCore();

private:
    void subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array);
	void subActorCallback(const carla_msgs::CarlaActorList &in_actor_list);
    void subOdomCallback(const nav_msgs::Odometry &in_odom);
    void containerManage();
    void calcDimension(ras_carla::RasObject &in_obj);
    void calcPolygon(ras_carla::RasObject &in_obj);
    void subShiftCallback(const ras_carla::RasObject &in_msg);
};
