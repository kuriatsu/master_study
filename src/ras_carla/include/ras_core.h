#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <math.h>
#include "ras_lib.h"

#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"

class RasCore
{
private:
	ros::Publisher pub_obj;
	ros::Subscriber sub_carla_obj;
	ros::Subscriber sub_shift;

    int keep_time;
	float recognize_distance;
    std::unordered_map<int, ras_carla::RasObject> obj_map;

public:
	RasCore();

private:
	void subObjCallback(derived_object_msgs::ObjectArray &in_obj_array);
	void containerManage();
	void subShiftCallback(const ras_carla::RasObject &in_msg);
    void subOdomCallback(const nav_msgs::Odometry &in_odom);
    void calcPolygon(ras_carla::RasObject &in_obj);
    void calcDimention(ras_carla::RasObject &in_obj);
};
