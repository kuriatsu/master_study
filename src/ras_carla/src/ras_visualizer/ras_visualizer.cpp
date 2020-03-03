#include <ros/ros.h>
#include "ras_visualizer.h"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

RasVisualizer::RasVisualizer(): marker_scale(1.0)
{
	ros::NodeHandle n;
    server.reset(new interactive_markers::InteractiveMarkerServer("ras_visualizer_node"));

    sub_obj = n.subscribe("/managed_objects", 5, &RasVisualizer::subObjCallback, this);
    sub_wall = n.subscribe("/wall_object", 5, &RasVisualizer::subWallCallback, this);
	// sub_vehicle_info = n.subscribe("/ego_vehicle/", 5, &RasVisualizer::subVehicleInfoCallback, this);
	// sub_erase_signal = n.subscribe("/erase_signal", 5, &RasVisualizer::erase_signal_callback, this);
    pub_fb_obj = n.advertise<ras_carla::RasObject>("/feedback_object", 5);
    pub_marker = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/object_marker", 5);
    // pub_marker = n.advertise<jsk_recognition_msgs::PolygonArray>("/object_polygon", 5);
    // pub_wall = n.advertise<visualization_msgs::Marker>("/wall_marker", 1);
	pub_wall = n.advertise<jsk_rviz_plugins::Pictogram>("/wall_marker", 1);
    pub_pictgram = n.advertise<jsk_rviz_plugins::PictogramArray>("/pictogram", 5);
}


RasVisualizer::~RasVisualizer()
{
    server.reset();
}

void RasVisualizer::subObjCallback(const ras_carla::RasObjectArray &in_obj_array)
{
    server->clear();
    jsk_recognition_msgs::BoundingBoxArray marker_array;
    // jsk_recognition_msgs::PolygonArray polygon_array;
    // visualization_msgs::MarkerArray marker_array;
    jsk_rviz_plugins::PictogramArray pictogram_array;
    for (auto e : in_obj_array.objects)
    {
        if (e.is_interaction)
        {
            createInteractiveMarker(e);
            if (e.is_important)
            {
                pictogram_array.pictograms.emplace_back(createPictogram(e, 0));
                pictogram_array.pictograms.emplace_back(createPictogram(e, 1));
                pictogram_array.pictograms.emplace_back(createPictogram(e, 2));
            }
        }
        // else
        // {
        //     marker_array.boxes.emplace_back(createMarker(e));
        //    // createMarker(e, polygon_array);
        // }
        marker_array.boxes.emplace_back(createMarker(e));
    }

    marker_array.header = in_obj_array.header;
    pictogram_array.header = in_obj_array.header;

    server->applyChanges();
    pub_marker.publish(marker_array);
    pub_pictgram.publish(pictogram_array);
}


void RasVisualizer::subWallCallback(const ras_carla::RasObject &in_obj)
{
    jsk_rviz_plugins::Pictogram pictogram;
    pictogram = createPictogram(in_obj, 3);
    pub_wall.publish(pictogram);
    // visualization_msgs::Marker marker;
    // marker = createMarker(in_obj);
    // pub_wall.publish(marker);
}


jsk_recognition_msgs::BoundingBox RasVisualizer::createMarker(const ras_carla::RasObject &in_obj)
{
    jsk_recognition_msgs::BoundingBox marker;

    marker.header = in_obj.object.header;
    marker.label = in_obj.object.id;

    marker.pose = in_obj.object.pose;
    marker.dimensions.x = in_obj.object.shape.dimensions[0];
    marker.dimensions.y = in_obj.object.shape.dimensions[1];
    marker.dimensions.z = in_obj.object.shape.dimensions[2];

    if (in_obj.object.classification == derived_object_msgs::Object::CLASSIFICATION_CAR || in_obj.object.classification == derived_object_msgs::Object::CLASSIFICATION_BARRIER)
    {
        marker.pose.position.z += in_obj.object.shape.dimensions[2] / 2;
    }

    marker.value = (in_obj.is_important) ? 100.0 : 50.0;
    return marker;
}


// visualization_msgs::Marker RasVisualizer::createMarker(const ras_carla::RasObject &in_obj)
// {
//     visualization_msgs::Marker marker;
//
//     marker.header = in_obj.object.header;
//     marker.ns = "ras";
//     marker.id = in_obj.object.id;
//
//     marker.type = visualization_msgs::Marker::CUBE;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose = in_obj.object.pose;
//     marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
//     marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
//     marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2];
//
//     if (in_obj.object.classification == derived_object_msgs::Object::CLASSIFICATION_CAR || in_obj.object.classification == derived_object_msgs::Object::CLASSIFICATION_BARRIER)
//     {
//         marker.scale.z += marker_scale*in_obj.object.shape.dimensions[2];
//     }
//
//     if (in_obj.is_important)
// 	{
// 		marker.color.r = 1;
// 		marker.color.g = 0;
// 	}
// 	else
// 	{
// 		marker.color.r = 0;
// 		marker.color.g = 1;
// 	}
//     marker.color.b = 0;
//     marker.color.a = 0.5;
//
//     marker.lifetime = ros::Duration(0.1);
//     return marker;
// }


// void RasVisualizer::createMarker(const ras_carla::RasObject &in_obj, jsk_recognition_msgs::PolygonArray &polygon_array)
// {
//     jsk_recognition_msgs::Polygon polygon;
//     geometry_msgs::Point32 point;
//     float obj_yaw = Ras::quatToYaw(in_obj.object.pose.orientation);
//     float radius = in_obj.object.shape.dimensions[0] / 2;
//     geometry_msgs::point obj_position = in_obj.object.pose.position;
//
//     point.x = radius * cos(obj_yaw) + obj_position.x;
//     point.y = radius * sin(obj_yaw) + obj_position.y;
//     point.z = obj_position.z;
//     polygon.points.emplace_back(point);
//
//     point.x = - radius * cos(obj_yaw) + obj_position.x;
//     point.y = - radius * sin(obj_yaw) + obj_position.y;
//     point.z = obj_position.z;
//     polygon.points.emplace_back(point);
//
//     point.x = radius * cos(obj_yaw) + obj_position.x;
//     point.y = radius * sin(obj_yaw) + obj_position.y;
//     point.z = obj_position.z + in_obj.object.shape.dimensions[3];
//     polygon.points.emplace_back(point);
//
//     point.x = - radius * cos(obj_yaw) + obj_position.x;
//     point.y = - radius * sin(obj_yaw) + obj_position.y;
//     point.z = obj_position.z + in_obj.object.shape.dimensions[3];
//     polygon.points.emplace_back(point);
//
//     polygon.header = in_obj.object.header;
//
//     polygon_array.polygons.emplace_back(polygon);
//     polygon_array.labels.emplace_back(in_obj.object.id);
// }

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

	if (in_obj.is_important)
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
    marker.color.a = 0.0;

    // marker.lifetime = ros::Duration(2.0);
    control.markers.push_back(marker);
}


jsk_rviz_plugins::Pictogram RasVisualizer::createPictogram(const ras_carla::RasObject &in_obj, const int &type)
{
    jsk_rviz_plugins::Pictogram pictogram;
    geometry_msgs::Pose arrow_pose;
    float arrow_len;
    std::map <int, std::string> message
    {
        {4,"fa-user"},
        {6, "fa-car"},
        {10, "block"}
        // {10, "traffic-cone"}
    };

    pictogram.header.stamp = ros::Time::now();
    pictogram.color.a = 1.0;
    pictogram.ttl = 0.1;

    switch (type)
    {
        case 0:
        {

            pictogram.header.frame_id = "base_link";
            pictogram.pose = Ras::tfTransformer(in_obj.object.pose, in_obj.object.header.frame_id, "base_link");
            pictogram.pose.position.z += 2.0;
            pictogram.pose.orientation.x = 0.0;
            pictogram.pose.orientation.y = 1.0;
            pictogram.pose.orientation.z = 0.0;
            pictogram.pose.orientation.w = -1.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 0.0;
            pictogram.color.b = 0.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::JUMP;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = "fa-angle-double-down";
            pictogram.speed = 5;
            pictogram.size = 3;
            break;
        }

        case 1:
        {

            pictogram.header.frame_id = "base_link";
            pictogram.pose = Ras::tfTransformer(in_obj.object.pose, in_obj.object.header.frame_id, "base_link");
            pictogram.pose.position.z += 5.0;
            pictogram.pose.orientation.x = 0.0;
            pictogram.pose.orientation.y = 1.0;
            pictogram.pose.orientation.z = 0.0;
            pictogram.pose.orientation.w = -1.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 1.0;
            pictogram.color.b = 1.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = message[in_obj.object.classification];
            pictogram.size = 6;
            // pictogram.character = message[in_obj.object.classification];
            break;
        }

        case 2:
        {

            pictogram.header.frame_id = "base_link";
            geometry_msgs::Pose arrow_pose = in_obj.object.pose;
            arrow_pose.position.x += 8.0;
            arrow_pose = Ras::tfTransformer(arrow_pose, in_obj.object.header.frame_id, "base_link");
            arrow_len = sqrt(pow(arrow_pose.position.x, 2) + pow(arrow_pose.position.y, 2));
            pictogram.pose.orientation.x = 0.0;
            pictogram.pose.orientation.y = 0.0;
            pictogram.pose.orientation.z = -arrow_pose.position.y / (arrow_len * 2);
            pictogram.pose.orientation.w = - arrow_pose.position.x / (arrow_len);
            pictogram.pose.position.x = 8.0;
            pictogram.pose.position.y = 0.0;
            pictogram.pose.position.z = 0.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 0.0;
            pictogram.color.b = 0.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = "fa-long-arrow-up";
            pictogram.size = 3;
            break;
        }

        case 3:
        {
            pictogram.header.frame_id = "map";
            pictogram.pose = in_obj.object.pose;
            pictogram.pose.position.z = 1.0;
            pictogram.pose.orientation.x = -1.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 0.6;
            pictogram.color.b = 0.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = message[in_obj.object.classification];
            pictogram.size = 5;
            break;
        }
    }

    return pictogram;
}

void RasVisualizer::intMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ras_carla::RasObject feedback_obj;
    std::istringstream sis;

    std::istringstream(feedback->marker_name) >> feedback_obj.object.id;
    pub_fb_obj.publish(feedback_obj);
}
