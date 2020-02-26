#include <ros/ros.h>
#include "ras_visualizer.h"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

RasVisualizer::RasVisualizer(): marker_scale(1.0)
{
	ros::NodeHandle n;
    server.reset(new interactive_markers::InteractiveMarkerServer("ras_visualizer_node"));

    sub_obj = n.subscribe("/managed_objects", 5, &RasVisualizer::subObjCallback, this);
	// sub_vehicle_info = n.subscribe("/ego_vehicle/", 5, &RasVisualizer::subVehicleInfoCallback, this);
	// sub_erase_signal = n.subscribe("/erase_signal", 5, &RasVisualizer::erase_signal_callback, this);
    pub_fb_obj = n.advertise<ras_carla::RasObject>("/feedback_info", 5);
	pub_marker = n.advertise<visualization_msgs::MarkerArray>("/ras_marker", 5);
}


RasVisualizer::~RasVisualizer()
{
    server.reset();
}

void RasVisualizer::subObjCallback(const ras_carla::RasObjectArray &in_obj_array)
{
    server->clear();
    visualization_msgs::MarkerArray marker_array;
    // ROS_INFO("visualezer subscribed");
    for (auto itr : in_obj_array.objects)
    {
        if (itr.is_interaction)
        {
            createInteractiveMarker(itr);
        }
        else
        {
            marker_array.markers.emplace_back(createMarker(itr));
        }
    }
    server->applyChanges();
    pub_marker.publish(marker_array);
}

visualization_msgs::Marker RasVisualizer::createMarker(const ras_carla::RasObject &in_obj)
{
    visualization_msgs::Marker marker;

    marker.header = in_obj.object.header;
    marker.ns = "ras";
    marker.id = in_obj.object.id;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = in_obj.object.pose;
    marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
    marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
    marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2];

    if (marker.id == 0)
    {
        marker.color.r = 1;
        marker.color.g = 0;
    }
    else
    {
        marker.color.r = 0;
        marker.color.g = 1;
    }
    marker.color.b = 0;
    marker.color.a = 0.3;

    marker.lifetime = ros::Duration(0.1);
    return marker;
}


void RasVisualizer::createInteractiveMarker(ras_carla::RasObject &in_obj)
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
    int_marker.pose.position.x = in_obj.object.pose.position.x;
    int_marker.pose.position.y = in_obj.object.pose.position.y;

    setMarkerControl(int_marker, in_obj);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&RasVisualizer::intMarkerCallback, this, _1));
}


void RasVisualizer::setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras_carla::RasObject &in_obj)
{
	visualization_msgs::InteractiveMarkerControl control;

	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    setMarkerToMarkerControl(control, in_obj);
    int_marker.controls.push_back(control);
}


void RasVisualizer::setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const ras_carla::RasObject &in_obj)
{
    visualization_msgs::Marker marker;

    marker.ns = "ras";
    marker.id = in_obj.object.id;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
    marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
    marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2];

	if (!in_obj.touch)
	{
		marker.color.r = 1;
		marker.color.g = 0;
	}
	else
	{
		marker.color.r = 0;
		marker.color.g = 1;
	}
    marker.color.b = 0;
    marker.color.a = 0.3;

    // marker.lifetime = ros::Duration(2.0);
    control.markers.push_back(marker);
}


void RasVisualizer::intMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ras_carla::RasObject feedback_obj;
    std::istringstream sis;

    std::istringstream(feedback->marker_name) >> feedback_obj.object.id;
    pub_fb_obj.publish(feedback_obj);
}
