#include <ros/ros.h>
#include "ras_autoware_connector.h"

RasAutowareConnector::RasAutowareConnector()//: keep_time(2)
{
    ros::NodeHandle n;

    sub_obj = n.subscribe("/managed_objects", 5, &RasAutowareConnector::subObjCallback, this);
    pub_obj = n.advertise<autoware_msgs::DetectedObjectArray>("/tracked_objects", 5);
}


void RasAutowareConnector::subObjCallback(ras_carla::RasObjectArray in_obj_array)
{
    autoware_msgs::DetectedObjectArray out_obj_array;
    autoware_msgs::DetectedObject out_obj;
    ras_carla::RasObject in_obj;
    out_obj_array.header = in_obj_array.header;

    for (size_t index = 0; index < in_obj_array.objects.size(); index++)
    {
        in_obj = in_obj_array.objects[index];

        out_obj.header = in_obj.object.header;
        out_obj.id = in_obj.object.id;
        // out_obj.label = in_obj.object.label;
        out_obj.score = in_obj.object.classification_certainty;
        // out_obj.color = color;
        out_obj.valid = false;
        out_obj.space_frame = "";
        out_obj.pose = in_obj.object.pose;
        out_obj.dimensions.x = in_obj.object.shape.dimensions[0];
        out_obj.dimensions.y = in_obj.object.shape.dimensions[1];
        out_obj.dimensions.z = in_obj.object.shape.dimensions[2];
        // out_obj.valiance = {0.0, 0.0, 0.0};
        out_obj.velocity = in_obj.object.twist;
        out_obj.acceleration.linear.x = in_obj.object.accel.linear.x;
        out_obj.acceleration.linear.y = in_obj.object.accel.linear.y;
        out_obj.acceleration.linear.x = in_obj.object.accel.linear.z;
        out_obj.acceleration.angular.x = in_obj.object.accel.angular.x;
        out_obj.acceleration.angular.y = in_obj.object.accel.angular.y;
        out_obj.acceleration.angular.z = in_obj.object.accel.angular.z;
        // out_obj.pointcloud = in_obj.object.pose;
        out_obj.convex_hull.polygon = in_obj.object.polygon;
        out_obj.pose.position.x = in_obj.object.pose.position.x + in_obj.shift_x;
        out_obj.pose.position.y = in_obj.object.pose.position.y + in_obj.shift_y;
        // out_obj.candidate_trajectories = in_obj.object.pose;
        out_obj.pose_reliable = true;
        out_obj.velocity_reliable = true;
        out_obj.acceleration_reliable = false;
        // out_obj.image_frame = true;
        out_obj.x = 0;
        out_obj.y = 0;
        out_obj.width = 0;
        out_obj.height = 0;
        out_obj.angle = 0.0;
        // out_obj.roi_image = true;
        out_obj.indicator_state = 0;
        out_obj.behavior_state = 0;
        // out_obj.user_defined_info = true;

        out_obj_array.objects.push_back(out_obj);
    }

    pub_obj.publish(out_obj_array);
}
