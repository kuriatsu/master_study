#include <ros/ros.h>
#include "ras_autoware_connector.h"

RasAutowareConnector::RasAutowareConnector(): polygon_interval(0.5)//, keep_time(2)
{
    ros::NodeHandle n;

    sub_obj = n.subscribe("/managed_objects", 5, &RasAutowareConnector::subObjCallback, this);
    pub_obj = n.advertise<autoware_msgs::DetectedObjectArray>("/tracked_objects", 5);
    // pub_polygon = n.advertise<geometry_msgs::PolygonStamped>("/ras_polygon", 10);
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
        out_obj.convex_hull = calcPolygon(in_obj);
        // pub_polygon.publish(out_obj.convex_hull);
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


geometry_msgs::PolygonStamped RasAutowareConnector::calcPolygon(ras_carla::RasObject &in_obj)
{
    geometry_msgs::Point32 point;
    geometry_msgs::PolygonStamped polygon_stamped;
    float x, y;

    polygon_stamped.header = in_obj.object.header;

    switch(in_obj.object.classification)
    {
        case 4:
        {
            int polygon_num = (in_obj.object.shape.dimensions[0] * M_PI) / polygon_interval;

            for (int i = 0; i < polygon_num; i++)
            {
                x = in_obj.object.shape.dimensions[0] * 0.5 * cos(2 * M_PI * i / polygon_num);
                y = in_obj.object.shape.dimensions[1] * 0.5 * sin(2 * M_PI * i / polygon_num);
                // std::cout << "pol num" << i << "/" << polygon_num << std::endl;
                // std::cout << "dimensions x:" << in_obj.object.shape.dimensions[0] << " y:" << in_obj.object.shape.dimensions[0] << std::endl;
                point.x = x + in_obj.object.pose.position.x + in_obj.shift_x;
                point.y = y + in_obj.object.pose.position.y + in_obj.shift_y;
                // std::cout << "x:" << point.x << " y:" << point.y << std::endl;
                point.z = 0.0;
                polygon_stamped.polygon.points.push_back(point);
            }
            break;
        }

        case 6:
        {
            int polygon_num_x = in_obj.object.shape.dimensions[0] / polygon_interval;
            int polygon_num_y = in_obj.object.shape.dimensions[1] / polygon_interval;
            double yaw = Ras::quatToYaw(in_obj.object.pose.orientation);

            for (int i = 0; i < polygon_num_x; i++)
            {
                x = ( -in_obj.object.shape.dimensions[0] / 2 + polygon_interval * i );
                y = ( in_obj.object.shape.dimensions[1] / 2 );
                point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                point.z = 0.0;
                polygon_stamped.polygon.points.push_back(point);

                y = ( -in_obj.object.shape.dimensions[1] / 2 );
                point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon_stamped.polygon.points.push_back(point);
            }

            for (int i = 0; i < polygon_num_y; i++)
            {
                x = ( in_obj.object.shape.dimensions[0] / 2 );
                y = ( -in_obj.object.shape.dimensions[1] / 2 + polygon_interval * i );
                point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                point.z = 0.0;
                polygon_stamped.polygon.points.push_back(point);

                x = ( -in_obj.object.shape.dimensions[0] / 2 );
                point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon_stamped.polygon.points.push_back(point);
            }
            break;
        }

        default:
        break;
    }

    return polygon_stamped;
}
