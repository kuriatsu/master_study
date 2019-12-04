#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <cmath>

#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"
#include "ras_visualizer.h"
// #include "std_msgs/Int32.h"


boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


class ObstacleVisualizer
{
private:

        ros::Subscriber sub_obj;
        // ros::Subscriber sub_erase_signal;
        ros::Subscriber sub_vehicle_info;
        ros::Publisher pub_shift;
        std::vector<uint32_t> id_vec;
        float marker_scale;

public:
	ObstacleVisualizer();
	void sync_jsk_box();

private:
        void subObjCallback(const ras_carla::RasObjectArray &in_obj_array);
        void subVehicleInfoCallback(condt);
        // void erase_signal_callback(const std_msgs::Int32 &in_msg);
        void shiftFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void createInteractiveMarker(const ras_carla::RasObject &in_obj);
        void setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras_carla::RasObject &in_obj);
        void setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const ras_carla::RasObject &in_obj);
};
