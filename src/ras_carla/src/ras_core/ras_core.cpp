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
	sub_shift = n.subscribe("/control_info", 10, &RasCore::subShiftCallback, this);
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
    auto itr_of_ego_wp;
	m_ego_pose = in_odom.pose.pose;
	m_ego_twist = in_odom.twist.twist;

	// find closest waypoint from m_waypoints
	for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
	{
		dist = pow(m_ego_pose.position.x - itr->pose.position.x, 2) + pow(m_ego_pose.position.y - itr->pose.position.y, 2);
		if (min_thres_pow > dist)
		{
			itr_of_ego_wp = itr;
			min_thres_pow = dist;
		}
	}
    m_itr_of_ego_wp = itr_of_ego_wp;
    m_itr_of_stoppable_wp = m_itr_of_ego_wp + (int)((pow(m_ego_twist.linear.x * 3.6, 2) / (254 * 0.7)) / m_wp_interval);
}


void RasCore::subTrajectoryCallback(const autoware_msgs::LaneArray &in_array)
{
	for (auto itr : in_array.lanes[0].waypoints)
	{
		m_wps_vec.emplace_back(itr.pose);
	}
	m_wp_interval = sqrt(pow(m_wps_vec[0].position.x - m_wps_vec[1].position.x, 2) + pow(m_wps_vec[0].position.y - m_wps_vec[1].position.y, 2));
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
			// selected_obj.is_front = (in_obj_pose.position.x > 0.5) ? true : false;
			// selected_obj.is_same_lane = (fabs(in_obj_pose.position.y) > 0.5) ? true : false;
            selected_obj.cross_wp_list = setCrossWp(selected_obj);
		}
	}
	takeAttendance();
}


std::list<int> RasCore::setCrossWp(ras_carla::RasObject &obj)
{
    std::list<int> cross_wp_list;
    switch(obj.object.classification)
    {
        case 4:
        {
            std::vector<float> obj_closest_wp_vec;
            float min_dist_of_wp_and_obj = m_max_vision, dist_of_wp_and_obj, inner_prod, dist_of_wp_and_ego, closest_vec_x, closest_vec_y, target_vec_x, target_vec_y;
            auto itr_of_closest_wp_from_obj, itr_of_cross_wp;

            // find closest waypoint from object
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                dist_of_wp_and_obj = sqrt(pow(obj.object.pose.position.x - itr->position.x, 2) + pow(obj.object.pose.position.y - itr->position.y, 2));
                if (dist_of_wp_and_obj < min_dist_of_wp_and_obj)
                {
                    min_dist_of_wp_and_obj = dist_of_wp_and_obj;
                    itr_of_closest_wp_from_obj = itr;
                }
            }

            closest_vec_x = itr_of_closest_wp_from_obj->position.x - obj.object.pose.position.x;
            closest_vec_y = itr_of_closest_wp_from_obj->position.y - obj.object.pose.position.y;

            // judge wheather the closest_wp should be considered
            dist_of_wp_and_ego = m_wp_interval * (std::distance(m_wps_vec.begin(), itr_of_closest_wp_from_obj) - std::distance(m_wps_vec.begin(), m_itr_of_ego_wp));
            if (dist_of_wp_and_ego > 0.0 && min_dist_of_wp_and_obj < dist_of_wp_and_ego && min_dist_of_wp_and_obj / obj.object.twist.linear.x < dist_of_wp_and_ego / m_ego_twist.linear.x)
            {
                cross_wp_list.emplace_back(obj.object.id);
            }

            // find vertical way of the object from the way to the closest waypoint
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                target_vec_x = itr->position.x - obj.object.pose.position.x;
                target_vec_y = itr->position.y - obj.object.pose.position.y;
                dist_of_wp_and_obj = sqrt(pow(target_vec_x, 2) + pow(target_vec_y, 2));
                inner_prod = target_vec_x * closest_vec_x + target_vec_y * closest_vec_y; // inner prod of closest_wp-obj vec and target_wp-obj vec

                if (inner_prod > 0.1 * dist_of_wp_and_obj * min_dist_of_wp_and_obj || itr == m_wps_vec.begin())
                {
                    continue;
                }
                // inner prod of neibour waypoint vec and obj-target_wp vec
                inner_prod = (itr->position.x - (itr-1)->position.x) * target_vec_x + (itr->position.y - (itr-1)->position.y) * target_vec_y;
                // judge wheather the found other way cross the ego_path vertically
                if (inner_prod < 0.98 * dist_of_wp_and_obj * m_wp_interval)
                {
                    continue;
                }
                // judge wheather the wp should be considered
                dist_of_wp_and_ego = m_wp_interval * (std::distance(m_wps_vec.begin(), itr) - std::distance(m_wps_vec.begin(), m_itr_of_ego_wp));
                if (dist_of_wp_and_obj > 0.0 && dist_of_wp_and_obj < dist_of_wp_and_ego && dist_of_wp_and_obj / obj.object.twist.linear.x < dist_of_wp_and_ego / m_ego_twist.linear.x)
                {
                    cross_wp_list.emplace_back(obj.object.id);
                }
            }
        }

        case 6:
        {
            float inner_prod, obj_vec_x, obj_vec_y, obj_wp_vec_x, obj_wp_vec_y, obj_vec_len, obj_wp_len;

            for (auto itr = m_itr_of_ego_wp; itr != m_wps_vec.end(); itr ++)
            {
                obj_vec_len = pow(obj.object.twist.linear.x * 3.6, 2) / (254 * 0.7);
                obj_vec_x = obj_vec_len * cos(Ras::quatToYaw(obj.object.pose.orientation));
                obj_vec_y = obj_vec_len * sin(Ras::quatToYaw(obj.object.pose.orientation));
                obj_wp_vec_x = itr->positon.x - obj.object.pose.position.x;
                obj_wp_vec_y = itr->positon.y - obj.object.pose.position.y;
                obj_wp_len = sqrt(pow(obj_wp_vec_x, 2) + pow(obj_wp_vec_y, 2));

                inner_prod = obj_vec_x * obj_wp_vec_x + obj_vec_y * obj_wp_vec_y;
                if (inner_prod < obj_vec_len * obj_wp_len * 0.98)
                {
                    cross_wp_list.emplace_back(obj.object.id);
                    break;
                }
            }
        }
    }
    return cross_wp_list.sort();
}


void RasCore::takeAttendance()
{
	std::vector<int> erase_key_vec, critical_obj_id_vec;
	ras_carla::RasObjectArray obj_array;
    ras_carla::RasObject wall;
    int min_dist_of_wp_from_ego = m_wps_vec.size(), dist_of_wp_from_ego, closest_wp;

	for(auto itr : m_obj_map)
	{
		// erace old object
		if ((ros::Time::now() - itr.second.object.header.stamp) > ros::Duration(m_keep_time))
		{
            erase_key_vec.emplace_back(itr.first);
            continue;
		}
        // get closest stop waypoint and effective obstacles
        for (size_t i = itr.socond.touch; i < itr.second.cross_wp_list.size(); i++)
        {
            dist_of_wp_from_ego = itr.second.cross_wp_list[i] - std::distance(m_wps_vec.begin(), m_itr_of_ego_wp);
            // if closer waypoint is found, clear vector and insert new one
            if (dist_of_wp_from_ego < min_dist_of_wp_from_ego)
            {
                critical_obj_id_vec.clear();
                min_dist_of_wp_from_ego = dist_of_wp_from_ego;
                critical_obj_id_vec.emplace_back(itr.second.object.id);
                closest_wp = itr.second.cross_wp_list[i];
            }
            // if critical obstacle is found at second time, add it to vector
            else if (dist_of_wp_from_ego == min_dist_of_wp_from_ego)
            {
                critical_obj_id_vec.emplace_back(itr.second.object.id);
            }
        }
	}
    // erace old object
    for (auto itr : erase_key_vec)
    {
        m_obj_map.erase(itr);
    }

    // judge wheather the object is critical or not and add to output list
    for (auto itr : m_obj_map)
    {
        itr.second.is_interaction = (critical_obj_id_vec.find(itr.second.object.id) != critical_obj_id_vec.end()) ? true : false;
        obj_array.objects.emplace_back(itr.second);
    }

    // finally add wall
    wall.object.header.stamp = ros::Time::now();
    wall.object.header.frame_id = "map";
    wall.object.id = 8888;
    wall.object.pose = m_wps_vec[closest_wp];
    wall.object.shape.type = shape_msgs::SolidPrimitive::BOX;
    wall.object.shape.dimentions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
    wall.object.shape.dimentions[shape_msgs::SolidPrimitive::BOX_Y] = 5.0;
    wall.object.shape.dimentions[shape_msgs::SolidPrimitive::BOX_Z] = 2.0;
    wall.is_interaction = false;
    obj_array.object.emplace_back(wall);

    // publish
	obj_array.header.stamp = ros::Time::now();
	obj_array.header.frame_id = "map";
	pub_obj.publish(obj_array);

}


void RasCore::subShiftCallback(const ras_carla::RasObject &in_msg)
{
	int id = in_msg.object.id;
	if (m_obj_map.find(id) != m_obj_map.end())
	{
        ras_carla::RasObject &obj = m_obj_map[id];
		std::cout << "touched" << std::endl;
        obj.touch ++;
		containerManage();
	}
}
