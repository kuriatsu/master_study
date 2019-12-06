#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore():polygon_interval(0.25), recognize_distance(50.0), keep_time(2.0)
{
	ros::NodeHandle n;

	sub_carla_obj = n.subscribe("/carla/objects", 5, &RasCore::subObjCallback, this);
	sub_shift = n.subscribe("/shifted_info", 10, &RasCore::subShiftCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &RasCore::subOdomCallback, this);
	pub_obj = n.advertise<ras_carla::RasObjectArray>("/managed_objects", 5);
	// pub_erase = n.advertise<std_msgs::Int32>("/erase_signal", 1);
    obj_map.clear();
}


void RasCore::subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array)
{
	geometry_msgs::Pose obj_pose;
	ras_carla::RasObject ras_obj;

    std::cout << "subscribed objects" << std::endl;
	for (size_t index=0; index < in_obj_array.objects.size(); index++)
	{
		ras_obj.object = in_obj_array.objects[index];

		obj_pose = Ras::tfTransformer(ras_obj.object.pose, ras_obj.object.header.frame_id, "base_link");
		ras_obj.distance = sqrt(pow(obj_pose.position.x, 2) + pow(obj_pose.position.y, 2));
		if (ras_obj.distance < recognize_distance)
		{
			obj_map[ras_obj.object.id].object = ras_obj.object;
			obj_map[ras_obj.object.id].distance = ras_obj.distance;
            // std::cout << ras_obj.object.id << " added" << std::endl;
		}
	}
	containerManage();
    std::cout << obj_map.size() << std::endl;
}


void RasCore::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	ego_pose = in_odom.pose.pose;
	ego_twist = in_odom.twist.twist;
}


void RasCore::containerManage()
{
    std::vector<int> erase_key_vec;

	ras_carla::RasObjectArray obj_array;

    for(auto itr = obj_map.begin(); itr != obj_map.end(); itr++)
    {
        std::cout << itr->first;
		if ((ros::Time::now() - itr->second.object.header.stamp) < ros::Duration(keep_time))
		{
            calcDimension(itr->second);
            calcPolygon(itr->second);
			obj_array.objects.push_back(itr->second);
            std::cout << " is left" << std::endl;
		}
		else
		{
            // auto erase_itr = obj_map.find(itr->first);
            // if (erase_itr != obj_map.end())
            // {
            erase_key_vec.push_back(itr->first);
            std::cout <<" is erased" << std::endl;
            std::cout << "time is " << ros::Time::now() - itr->second.object.header.stamp << std::endl;
            // }
		}
	}

    obj_array.header.stamp = ros::Time::now();
    obj_array.header.frame_id = "map";
	pub_obj.publish(obj_array);

    for (auto itr : erase_key_vec)
    {
        obj_map.erase(itr);
    }
}


void RasCore::calcDimension(ras_carla::RasObject &in_obj)
{
    float movable_dist;
    if (ego_twist.linear.x > 3.0)
    {
        movable_dist = in_obj.object.twist.linear.x * in_obj.distance / ego_twist.linear.x;

        switch(in_obj.object.classification)
        {
            case 4: //pedestrian
                in_obj.object.shape.type = 1;
                in_obj.object.shape.dimensions[0] = movable_dist;
                in_obj.object.shape.dimensions[1] = movable_dist;
                break;

            case 6: //car
                in_obj.object.shape.type = 1;
                in_obj.object.shape.dimensions[0] = movable_dist;
                in_obj.object.pose.position.x += 0.5 * movable_dist * cos(Ras::quatToYaw(in_obj.object.pose.orientation));
                in_obj.object.pose.position.y += 0.5 * movable_dist * sin(Ras::quatToYaw(in_obj.object.pose.orientation));
            break;

            default:
            break;
        }
    }
}


void RasCore::calcPolygon(ras_carla::RasObject &in_obj)
{
    geometry_msgs::Point32 polygon;
    int x, y;

    switch(in_obj.object.classification)
    {
        case 4:
        {
            int polygon_num = (in_obj.object.shape.dimensions[0] * M_PI) / polygon_interval;

            for (int i = 0; i < polygon_num; i++)
            {
                x = in_obj.object.shape.dimensions[0] * 0.5 * cos(2 * M_PI * i / polygon_num);
                y = in_obj.object.shape.dimensions[1] * 0.5 * sin(2 * M_PI * i / polygon_num);
                polygon.x = x + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = y + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon.z = 0.0;
                in_obj.object.polygon.points.push_back(polygon);
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
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon.z = 0.0;
                in_obj.object.polygon.points.push_back(polygon);

                y = ( -in_obj.object.shape.dimensions[1] / 2 );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                in_obj.object.polygon.points.push_back(polygon);
            }

            for (int i = 0; i < polygon_num_y; i++)
            {
                x = ( in_obj.object.shape.dimensions[0] / 2 );
                y = ( -in_obj.object.shape.dimensions[1] / 2 + polygon_interval * i );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                polygon.z = 0.0;
                in_obj.object.polygon.points.push_back(polygon);

                x = ( -in_obj.object.shape.dimensions[0] / 2 );
                polygon.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x + in_obj.shift_x;
                polygon.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y + in_obj.shift_y;
                in_obj.object.polygon.points.push_back(polygon);
            }
            break;
        }

        default:
            break;
    }
}


void RasCore::subShiftCallback(const ras_carla::RasObject &in_msg)
{
	int id = in_msg.object.id;
    ras_carla::RasObject obj;
    auto itr = obj_map.find(id);
    if (itr != obj_map.end())
    {
        obj = obj_map[id];
        std::cout << obj.shift_x << " is shifted"<< std::endl;
        obj.shift_x = in_msg.object.pose.position.x - obj_map[id].object.pose.position.x;
        obj.shift_y = in_msg.object.pose.position.y - obj_map[id].object.pose.position.y;
        obj.object.header.stamp = ros::Time::now();
        obj_map[id] = obj;
        containerManage();
    }
}
