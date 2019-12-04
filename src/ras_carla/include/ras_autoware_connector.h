#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "ras_carla/RasObjectArray.h"
#include "ras_carla/RasObject.h"


class RasAutowareConnector{

private:
    ros::Subscriber sub_obj;
    ros::Publisher pub_obj;

    public :
    RasAutowareConnector();

    private :
    void subObjCallback(const ras_carla::RasObjectArray &in_obj_array);
};
