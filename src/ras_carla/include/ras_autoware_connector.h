#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "ras_carla/RasObjectArray.h"
#include "ras_carla/RasObject.h"


class RasAutowareConnector{

private:
    ros::Subscriber sub_obj;
    ros::Publisher pub_obj;
    // ros::Publisher pub_polygon;
public :
    RasAutowareConnector();

private :
    void subObjCallback(ras_carla::RasObjectArray in_obj_array);
};
