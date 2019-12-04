#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore():polygon_interval(0.25)
{
	ros::NodeHandle n;

	sub_carla_obj = n.subscribe("/carla/objects", 5, &RasCore::subObjCallback, this);
	sub_shift = n.subscribe("/shifted_info", 5, &RasCore::subShiftCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &RasCore::subOdomCallback, this);
	pub_obj = n.advertise<ras_carla::RasObjectArray>("/managed_objects", 5);
	// pub_erase = n.advertise<std_msgs::Int32>("/erase_signal", 1);
}


void RasCore::subObjCallback(derived_object_msgs::ObjectArray &in_obj_array)
{
	geometry_msgs::Pose obj_pose;
	ras_carla::RasObject ras_obj;

	for (size_t index=0; index < in_obj_array.objects.size(); index++)
	{
		ras_obj.object = in_obj_array.objects[index];

		obj_pose = Ras::tfTransformer(ras_obj.pose, ras_obj.header.frame_id, "base_link");
		ras_obj.distance = sqrt(obj_pose.x**2 + obj_pose.y**2);;

		if (ras_obj.distance < recognize_distance)
		{
			obj_map[ras_obj.id].object = ras_obj.object;
			obj_map[ras_obj.id].distance = ras_obj.distance;
		}
	}
	containerManage();
}


void RasCore::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	ego_pose.pose = in_odom.pose.pose;
	ego_twist.twist = in_odom.twist.twist;
}


void RasCore::containerManage()
{
	ras_carla::RasObjectArray obj_array;

    for(auto itr = obj_map.begin(); itr != obj_map.end(); ++itr)
    {

		if ((ros.Time.now() - itr->object.header.stamp) > ros::Duration(keep_time))
		{
            calcDimention(*itr);
            calcPolygon(*itr);
			obj_array.append(*itr);
		}
		else
		{
			obj_map.erace(itr->object.id);
		}
	}
	pub_obj.publish(obj_array);
}


void RasCore::calcDimention(ras_carla::RasObject &in_obj)
{
    float movable_dist = in_obj.object.twist.linear.x * in_obj.distance / ego_twist.linear.x;

    switch(in_obj.object.classification)
    {
        case 4: //pedestrian
            in_obj.object.shape.type = 3;
            in_obj.object.shape.dimensions[0] = movable_dist;
            in_obj.object.shape.dimensions[1] = movable_dist;
            break;

        case 6: //car
            in_obj.object.shape.type = 1;
            in_obj.object.shape.dimensions[0] = movable_dist;
            break;

        default:
            break;
    }
}


void RasCore::calcPolygon(ras_carla::RasObject &in_obj)
{
    geometry_msgs::Point32 polygon;
    int x, y;

    switch(in_obj.object.classification)
    {
        case 4:
            int polygon_num = (in_obj.object.shape.dimensions[0] * M_PI) / polygon_interval;

            for (i = 0; i < polygon_num; i++)
            {
                x = in_obj.object.shape.dimensions[0] / 2 * cos(2 * M_PI * i / polygon_num);
                y = in_obj.object.shape.dimensions[1] / 2 * sin(2 * M_PI * i / polygon_num);
                polygon.x = x + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = y + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon.z = 0.0;
                in_obj.object.polygon.points.append(polygon);
            }
            break;

        case 6:
            int polygon_num_x = in_obj.object.shape.dimensions[0] / polygon_interval;
            int polygon_num_y = in_obj.object.shape.dimensions[1] / polygon_interval;
            double yaw = Ras::getRPY(in_obj.object.pose.orientation);

            for (i = 0; i < polygon_num_x; i++)
            {
                x = ( -in_obj.object.shape.dimensions[0] / 2 + polygon_interval * i );
                y = ( in_obj.object.shape.dimensions[1] / 2 );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon.z = 0.0;
                in_obj.object.polygon.points.append(polygon);

                y = ( -in_obj.object.shape.dimensions[1] / 2 );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                in_obj.object.polygon.points.append(polygon);
            }

            for (i = 0; i < polygon_num_y; i++)
            {
                x = ( in_obj.object.shape.dimensions[0] / 2 );
                y = ( -in_obj.object.shape.dimensions[1] / 2 + polygon_interval * i );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon.z = 0.0;
                in_obj.object.polygon.points.append(polygon);

                x = ( -in_obj.object.shape.dimensions[0] / 2 );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                in_obj.object.polygon.points.append(polygon);
            }
            break;

        default:
            break;
    }
}


void RasCore::subShiftCallback(const ras_carla::RasObject &in_msg)
{
	int id = in_msg.object.id;
    if (obj_map.find(id))
    {
        obj_map[id].shift_x = in_msg.object.pose.position.x - obj_map[id].object.pose.position.x;
        obj_map[id].shift_y = in_msg.object.pose.position.y - obj_map[id].object.pose.position.y;
        obj_map[id].object.header.stamp = ros::Time::now();
        containerManage();
    }
}
