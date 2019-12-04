#include <ros/ros.h>
#include "ras_visualizer.h"

RasVisualizer::RasVisualizer(): marker_scale(1.0)
{
	ros::NodeHandle n;

    sub_obj = n.subscribe("/managed_objects", 5, &RasVisualizer::subObjCallback, this);
	sub_vehicle_info = n.subscribe("/ego_vehicle/", 5, &RasVisualizer::subVehicleInfoCallback, this);
	// sub_erase_signal = n.subscribe("/erase_signal", 5, &RasVisualizer::erase_signal_callback, this);
	pub_shift = n.advertise<ras_carla::RasObject>("/shifted_info", 5);
}


void RasVisualizer::subObjCallback(const swipe_obstacles::detected_obstacle_array &in_obj_array)
{
    server->clear();
    // ROS_INFO("visualezer subscribed");

    for (size_t i=0; i< in_obj_array.obstacles.size(); i++)
    {
        createInteractiveMarker(in_obj_array.obstacles[i]);
    }

    server->applyChanges();
}


void RasVisualizer::createInteractiveMarker(const ras_carla::RasObject &in_obj)
{
    // for debag
    // std::cout <<"ss id is:" << in_obj.id << std::endl;
    std::stringstream ss;
    ss << in_obj.object.id;

	visualization_msgs::InteractiveMarker int_marker;
    // ROS_INFO_STREAM(in_obj);
	int_marker.header.frame_id = in_obj.object.header.frame_id;
	int_marker.name = ss.str();
	int_marker.scale = marker_scale;
    int_marker.pose = in_obj.object.pose;
    int_marker.pose.position.x = in_obj.object.pose.position.x + in_obj.shift_x;
	int_marker.pose.position.y = in_obj.object.pose.position.y + in_obj.shift_y;

    setMarkerControl(int_marker, in_obj);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&RasVisualizer::shiftFeedback, this, _1));
}


void RasVisualizer::setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras_carla::RasObject &in_obj)
{
	visualization_msgs::InteractiveMarkerControl control;

	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.orientation.w = 1;

    setMarkerToMarkerControl(control, in_obj);
	// control.markers.push_back(setMarkerToMarkerControl(in_obj));
    int_marker.controls.push_back(control);
}


void RasVisualizer::setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const swipe_obstacles::detected_obstacle &in_obj)
{
    visualization_msgs::Marker marker;

    marker.ns = "ras";
    marker.id = in_obj.object.id;

    if (in_obj.object.label == 4)
    {
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = marker_scale*in_obj.distance;
        marker.scale.y = marker_scale*in_obj.distance;
        marker.scale.z = marker_scale*0.5;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 0.6;
    }

    else if (in_obj.label == 5)
    {
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = marker_scale*15;
        marker.scale.y = marker_scale*15;
        marker.scale.z = marker_scale*1.3;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.6;
    }

    else if (in_obj.label == 6)
    {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = marker_scale*40;
        marker.scale.y = marker_scale*4.0;
        marker.scale.z = marker_scale*1.5;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.6;
    }
    // marker.lifetime = ros::Duration(2.0);
    control.markers.push_back(marker);
}


void RasVisualizer::shiftFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ras_carla::RasObject feedback_obj;
    std::istringstream sis;

    sis = std::istringstream(feedback->marker_name);
    feedback_obj.object.pose = feedback->pose;
    sis >> feedback_obj.object.id;
    pub_shift.publish(feedback_obj);
}

void RasVisualizer::subVehicleInfoCallback()
{
	
}
