#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <math.h>

#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"

class DetectorGT
{
private:
	ros::Publisher pub_obj;
	ros::Subscriber sub_carla_obj;

	geometry_msgs::Pose ego_pose;
    tf::TransformListener tf_listener;

public:
	DetectorGT();

private:
	void subObjCallback(const derived_object_msgs::ObjectArray &in_object_array);
	geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id);
};
