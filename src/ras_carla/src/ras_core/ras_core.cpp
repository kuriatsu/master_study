#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore()
{
	ros::NodeHandle n;

	sub_carla_obj = n.subscribe("/carla/objects", 5, &RasCore::subObjCallback, this);
	sub_shift = n.subscribe("/shifted_info", 5, &RasCore::subShiftCallback, this);
	pub_obj = n.advertise<ras_carla::RasObjectArray>("/managed_objects", 5);
	// pub_erase = n.advertise<std_msgs::Int32>("/erase_signal", 1);
}


void RasCore::subObjCallback(derived_object_msgs::ObjectArray &in_obj_array)
{
	geometry_msgs::Pose obj_pose;
	ras_carla::RasObject ras_obj;

	for (size_t index=0; index < in_obj_array.objects.size(); index++)
	{
		ras_obj.object = in_obj_array[index];

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


void RasCore::containerManage()
{
	ras_carla::RasObjectArray obj_array;

    for(auto itr = obj_map.begin(); itr != obj_map.end(); ++itr)
    {

		if ((ros.Time.now() - itr->object.header.stamp) > ros::Duration(keep_time))
		{
			obj_array.append(*itr);
		}
		else
		{
			obj_map.erace(itr->object.id);
		}
	}
	pub_obj.publish(obj_array);
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
