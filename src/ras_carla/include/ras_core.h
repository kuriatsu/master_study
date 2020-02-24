#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <nav_msgs/Odometry.h>
#include <shape_msgs/SolidPrimitive.h>

#include "ras_lib.h"
#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"
#include <ras_carla/rasConfig.h>

#include "carla_msgs/CarlaActorList.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/Waypoint.h"

#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <math.h>

class RasCore
{
private:
    ros::Publisher pub_obj;
	ros::Subscriber sub_carla_actor_list;
	ros::Subscriber sub_carla_obj;
    ros::Subscriber sub_shift;
	ros::Subscriber sub_odom;
    ros::Subscriber sub_trajectory;

    int m_keep_time;
    float m_max_vision;
    float m_min_vision;
	std::string m_ego_name;
    bool m_conservative_recognition;

    std::unordered_map<int, ras_carla::RasObject> m_obj_map;
    std::unordered_map<int, std::vector<int>> m_wp_obj_map;
    geometry_msgs::Pose m_ego_pose;
    geometry_msgs::Twist m_ego_twist;
    int m_ego_id;
    std::vector<geometry_msgs::Pose> m_wps_vec;
    int m_index_of_ego_wp;
    float m_wp_interval;

    dynamic_reconfigure::Server<ras_carla::rasConfig> server;
    dynamic_reconfigure::Server<ras_carla::rasConfig>::CallbackType server_callback;

public:
	RasCore();

private:
    void subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array);
	void subActorCallback(const carla_msgs::CarlaActorList &in_actor_list);
    void subOdomCallback(const nav_msgs::Odometry &in_odom);
    void containerManage();
    void calcDimension(ras_carla::RasObject &in_obj);
    void subShiftCallback(const ras_carla::RasObject &in_msg);
    void callbackDynamicReconfigure(ras_carla::rasConfig &config, uint32_t lebel);
    void subTrajectoryCallback(const autoware_msgs::LaneArray &in_array);
};
