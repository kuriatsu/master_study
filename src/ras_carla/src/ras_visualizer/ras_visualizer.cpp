#include <ros/ros.h>
#include "ras_visualizer.h"

ObstacleVisualizer::ObstacleVisualizer(): marker_scale(1.0)
{
	ros::NodeHandle n;

    sub_obj = n.subscribe("/managed_obstacles", 5, &ObstacleVisualizer::sub_obstacles_callback, this);
	sub_erase_signal = n.subscribe("/swipe_erase_signal", 5, &ObstacleVisualizer::erase_signal_callback, this);
	pub_shift = n.advertise<swipe_obstacles::detected_obstacle>("/shifted_info", 5);
}


void ObstacleVisualizer::erase_signal_callback(const std_msgs::Int32 &in_msg)
{
    if(in_msg.data)
    {
        // ROS_INFO_STREAM(in_msg.data);
        server->clear();
        server->applyChanges();
    }
}


void ObstacleVisualizer::sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    server->clear();
    // ROS_INFO("visualezer subscribed");

    for (size_t i=0; i< in_msgs.obstacles.size(); i++)
    {
        createInteractiveMarker(in_msgs.obstacles[i]);
    }
	// for (size_t i=0; i< in_msgs.obstacles.size(); i++)
    // {
    //     auto itr = std::find(id_vec.begin(), id_vec.end(), in_msgs[i].managed_id);
    //
    //     if(itr != id_vec.end())
    //     {
    //         server->setPose(*itr, calc_boxpose(in_msgs.obstacles[i].pose, in_msgs.obstacles[i].shift));
    //     }
    //     else
    //     {
    //         make_cube(in_msgs.obstacles[i]);
    //         id_vec.push_back(in_msgs.obstacles[i].managed_id);
    //     }
	// }
    //
    // for (auto i=prev_obj_ids.begin(); i != prev_obj_ids.end(), i++)
    // {
    //     server->erase(*i);
    // }
    // // idデータリスト更新,新データリストは空にシておく.
    // prev_obj_ids = current_obj_ids;
    server->applyChanges();

}


void ObstacleVisualizer::createInteractiveMarker(const swipe_obstacles::detected_obstacle &obstacle_info)
{
    // for debag
    // std::cout <<"ss id is:" << obstacle_info.id << std::endl;
    std::stringstream ss;
    ss << obstacle_info.id;

	visualization_msgs::InteractiveMarker int_marker;
    // ROS_INFO_STREAM(obstacle_info);
	int_marker.header.frame_id = "map";
	int_marker.name = ss.str();
	int_marker.scale = marker_scale;
    int_marker.pose = obstacle_info.pose;
    int_marker.pose.position.x = obstacle_info.pose.position.x + obstacle_info.shift_x;
	int_marker.pose.position.y = obstacle_info.pose.position.y + obstacle_info.shift_y;

    setMarkerControl(int_marker, obstacle_info);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&ObstacleVisualizer::shift_feedback, this, _1));
}


void ObstacleVisualizer::setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const swipe_obstacles::detected_obstacle &obstacle_info)
{
	visualization_msgs::InteractiveMarkerControl control;

	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.orientation.w = 1;

    setMarkerToMarkerControl(control, obstacle_info);
	// control.markers.push_back(setMarkerToMarkerControl(obstacle_info));
    int_marker.controls.push_back(control);
}


void ObstacleVisualizer::setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const swipe_obstacles::detected_obstacle &obstacle_info)
{
    visualization_msgs::Marker marker;

    marker.ns = "swipe_obstacles";
    marker.id = obstacle_info.id;

    if (obstacle_info.label == "person")
    {
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = marker_scale*obstacle_info.distance;
        marker.scale.y = marker_scale*obstacle_info.distance;
        marker.scale.z = marker_scale*1.0;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 0.7;
    }
    
    else if (obstacle_info.label == "bicycle")
    {
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = marker_scale*15;
        marker.scale.y = marker_scale*15;
        marker.scale.z = marker_scale*1.4;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.7;
    }
    else if (obstacle_info.label == "car")
    {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = marker_scale*40;
        marker.scale.y = marker_scale*4.0;
        marker.scale.z = marker_scale*1.7;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.7;
    }
    // marker.lifetime = ros::Duration(2.0);

    control.markers.push_back(marker);
}


void ObstacleVisualizer::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    swipe_obstacles::detected_obstacle feedback_obstacle;
    std::istringstream sis;

    sis = std::istringstream(feedback->marker_name);
    feedback_obstacle.pose = feedback->pose;
    sis >> feedback_obstacle.id;
    pub_shift.publish(feedback_obstacle);
}
