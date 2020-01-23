#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "ras_carla/RasObjectArray.h"
#include "ras_carla/RasObject.h"

#include "ras_lib.h"

class RasAutowareConnector{

private:
    ros::Subscriber sub_obj;
    ros::Publisher pub_obj;
    // ros::Publisher pub_polygon;
    float polygon_interval;
public :
    RasAutowareConnector();

private :
    void subObjCallback(ras_carla::RasObjectArray in_obj_array);
    geometry_msgs::PolygonStamped calcPolygon(ras_carla::RasObject &in_obj);
};
