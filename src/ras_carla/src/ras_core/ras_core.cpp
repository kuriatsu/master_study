#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore()
{
	ros::NodeHandle n;

	server_callback = boost::bind(&RasCore::callbackDynamicReconfigure, this, _1, _2);
	server.setCallback(server_callback);

	sub_carla_obj = n.subscribe("/carla/objects", 1, &RasCore::subObjCallback, this);
	sub_carla_actor_list = n.subscribe("/carla/actor_list", 1, &RasCore::subActorCallback, this);
	sub_trajectory = n.subscribe("/lane_waypoint_array", 5, &RasCore::subTrajectoryCallback, this);
	sub_shift = n.subscribe("/shifted_info", 10, &RasCore::subShiftCallback, this);
	sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &RasCore::subOdomCallback, this);
	pub_obj = n.advertise<ras_carla::RasObjectArray>("/managed_objects", 5);
	// pub_erase = n.advertise<std_msgs::Int32>("/erase_signal", 1);

	m_obj_map.clear();
}


void RasCore::callbackDynamicReconfigure(ras_carla::rasConfig &config, uint32_t lebel)
{
	m_conservative_recognition = config.conservative?true:false;
	m_max_vision = config.max_vision;
	m_min_vision = config.min_vision;
	m_keep_time = config.keep_time;
	m_ego_name = config.ego_name;
}


void RasCore::subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array)
{
	geometry_msgs::Pose in_obj_pose;
	ras_carla::RasObject ras_obj;

	for (size_t i=0; i < in_obj_array.objects.size(); i++)
	{
		ras_obj.object = in_obj_array.objects[i];

		in_obj_pose = Ras::tfTransformer(ras_obj.object.pose, ras_obj.object.header.frame_id, "base_link");
		ras_obj.distance = sqrt(pow(in_obj_pose.position.x, 2) + pow(in_obj_pose.position.y, 2));

		if (m_min_vision < ras_obj.distance && ras_obj.distance < m_max_vision && ras_obj.object.id != ego_id)
		{
			ras_carla::RasObject &selected_obj = m_obj_map[ras_obj.object.id];
			selected_obj.object = ras_obj.object;
			selected_obj.distance = ras_obj.distance;
			selected_obj.is_front = (in_obj_pose.position.x > 0.5) ? true : false;
			selected_obj.is_same_lane = (fabs(in_obj_pose.position.y) > 0.5) ? true : false;
		}
	}
	containerManage();
	// std::cout << m_obj_map.size() << std::endl;
}

// get ego_vehicle id to remove ego_object from obstacles
void RasCore::subActorCallback(const carla_msgs::CarlaActorList &in_actor_list)
{
	for (size_t i = 0; i < in_actor_list.actors.size(); i ++)
	{
		if (in_actor_list.actors[i].rolename == m_ego_name)
		{
			m_ego_id = in_actor_list.actors[i].id;
		}
	}
}

void RasCore::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	float min_thres_pow = 9.0, dist;
	m_ego_pose = in_odom.pose.pose;
	m_ego_twist = in_odom.twist.twist;

	// find closest waypoint from m_waypoints
	for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
	{
		dist = pow(m_ego_pose.position.x - itr->pose.position.x, 2) + pow(m_ego_pose.position.y - itr->pose.position.y, 2);
		if (min_thres_pow > dist)
		{
			m_index_of_ego_wp = std::distance(m_wps_vec.begin(), itr);
			min_thres_pow = dist;
		}
	}
}


void RasCore::subTrajectoryCallback(const autoware_msgs::LaneArray &in_array)
{
	for (auto itr : in_array.lanes[0].waypoints)
	{
		m_wps_vec.emplace_back(itr.pose);
	}
	m_wp_interval = sqrt(pow(m_wps_vec[0].position.x - m_wps_vec[1].position.x, 2) + pow(m_wps_vec[0].position.y - m_wps_vec[1].position.y, 2));
}


void RasCore::containerManage()
{
	std::vector<int> erase_key_vec;
	ras_carla::RasObjectArray obj_array;

	for(auto itr : m_obj_map)
	{
		// std::cout << itr->first;
		if ((ros::Time::now() - itr.second.object.header.stamp) < ros::Duration(m_keep_time))
		{
			findCrossPoint(itr.second);
			obj_array.objects.emplace_back(itr.second);
			// std::cout << " is left" << std::endl;
		}
		else
		{
			erase_key_vec.emplace_back(itr.first);
			// std::cout <<" is erased" << std::endl;
		}
	}

	obj_array.header.stamp = ros::Time::now();
	obj_array.header.frame_id = "map";
	pub_obj.publish(obj_array);

	for (auto itr : erase_key_vec)
	{
		m_obj_map.erase(itr);
	}
}


void RasCore::findCrossPoint(ras_carla::RasObject &obj)
{
	std::vector<int> found_wp_vec;

	switch(object.object.classification)
	{
		case 4:
		float shortest_dist_of_wp_and_obj = m_max_vision;
		int index_of_closest_wp_from_obj;

		for (size_t i = m_index_of_ego_wp; i < m_wps_vec.size(); i++)
		{
			float dist_of_wp_and_obj = sqrt(pow(obj.object.pose.position.x - m_wps_vec[index].pose.position.x, 2) + pow(obj.object.pose.position.y - m_wps_vec[index].pose.position.y, 2));
			if (dist_of_wp_and_obj < shortest_dist_of_wp_and_ego)
			{
				shortest_dist_of_wp_and_ego = dist_of_wp_and_obj;
				index_of_closest_wp_from_obj = (int)index;
			}
		}

		float dist_of_target_wp_and_ego = m_wp_interval * (index_of_closest_wp_from_obj - m_index_of_ego_wp);
		if (shortest_dist_of_wp_and_ego < dist_of_target_wp_and_ego && shortest_dist_of_wp_and_ego / obj.object.twist.linear.x < point_ego_dist / m_ego_twist.linear.x)
		{
			found_wp_vec.emplace_back(index_of_closest_wp_from_obj);

		}

		point_vec.emplace_back(std::distance(m_waypoints.begin(), m_wps_vec[index] - 1));
		is_descent = false;
		previous_dist = dist;

		break;
	}



}


void RasCore::calcDimension(ras_carla::RasObject &in_obj)
{
	float movable_dist, movable_vel, safety_dist;
	float clipped_ego_vel = m_ego_twist.linear.x > min_recognize_vel ? m_ego_twist.linear.x : min_recognize_vel;
	if(m_ego_twist.linear.x > min_recognize_vel && m_conservative_recognition)
	{

		switch(in_obj.object.classification)
		{
			case 4: //pedestrian
			// movable_vel = in_obj.object.twist.linear.x > min_recognize_vel ? in_obj.object.twist.linear.x : min_recognize_vel;
			// movable_dist = movable_vel * in_obj.distance / clipped_ego_vel;
			safety_dist = pow((m_ego_twist.linear.x+in_obj.object.twist.linear.x) * 3.6, 2) / (254 * 0.7);
			in_obj.object.shape.type = 1;
			in_obj.object.shape.dimensions[0] = safety_dist;
			in_obj.object.shape.dimensions[1] = safety_dist;
			break;

			case 6: //car
			{
				float inner_prod = cos(Ras::quatToYaw(m_ego_pose.orientation) - Ras::quatToYaw(in_obj.object.pose.orientation));
				in_obj.object.shape.type = 1;

				if (in_obj.is_front && inner_prod < 0 && in_obj.object.twist.linear.x > 0.1)
				{
					// movable_vel = in_obj.object.twist.linear.x;
					// movable_dist = movable_vel * in_obj.distance / clipped_ego_vel;
					safety_dist = pow((m_ego_twist.linear.x+in_obj.object.twist.linear.x) * 3.6, 2) / (254 * 0.7);
					in_obj.object.shape.dimensions[0] = safety_dist;
					in_obj.object.pose.position.x += 0.5 * safety_dist * cos(Ras::quatToYaw(in_obj.object.pose.orientation));
					in_obj.object.pose.position.y += 0.5 * safety_dist * sin(Ras::quatToYaw(in_obj.object.pose.orientation));
				}

				else if (!in_obj.is_front && !in_obj.is_just_back && inner_prod > 0 && in_obj.object.twist.linear.x - m_ego_twist.linear.x > 5.0)
				{
					movable_vel = in_obj.object.twist.linear.x;
					movable_dist = movable_vel * in_obj.distance / (in_obj.object.twist.linear.x - m_ego_twist.linear.x) ;
					in_obj.object.shape.dimensions[0] = movable_dist;
					in_obj.object.pose.position.x += 0.5 * movable_dist * cos(Ras::quatToYaw(in_obj.object.pose.orientation));
					in_obj.object.pose.position.y += 0.5 * movable_dist * sin(Ras::quatToYaw(in_obj.object.pose.orientation));
				}
				break;
			}

			default:
			break;
		}

	}
}


void RasCore::subShiftCallback(const ras_carla::RasObject &in_msg)
{
	int id = in_msg.object.id;
	ras_carla::RasObject obj;
	auto itr = m_obj_map.find(id);
	if (itr != m_obj_map.end())
	{
		obj = m_obj_map[id];
		std::cout << obj.shift_x << " is shifted"<< std::endl;
		obj.shift_x = in_msg.object.pose.position.x - m_obj_map[id].object.pose.position.x;
		obj.shift_y = in_msg.object.pose.position.y - m_obj_map[id].object.pose.position.y;
		obj.object.header.stamp = ros::Time::now();
		m_obj_map[id] = obj;
		containerManage();
	}
}
