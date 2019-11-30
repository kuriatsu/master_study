#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

#include "std_msgs/Int32.h"
#include <interactive_markers/interactive_marker_server.h>

#include <cmath>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


class ObstacleVisualizer
{
private:

        ros::Subscriber sub_obj;
        ros::Subscriber sub_erase_signal;
        ros::Publisher pub_shift;
        std::vector<uint32_t> id_vec;
        float marker_scale;

public:
	ObstacleVisualizer();
	void sync_jsk_box();

private:
        void sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void erase_signal_callback(const std_msgs::Int32 &in_msg);
        void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void createInteractiveMarker(const swipe_obstacles::detected_obstacle &in_msg);
        void setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const swipe_obstacles::detected_obstacle &obstacle_info);
        void setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const swipe_obstacles::detected_obstacle &obstacle_info);
        int id_vector_manager(const uint32_t &id);
};