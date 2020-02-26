#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore(): m_ego_wp(0)
{
	ros::NodeHandle n;

	server_callback = boost::bind(&RasCore::callbackDynamicReconfigure, this, _1, _2);
	server.setCallback(server_callback);

    sub_trajectory = n.subscribe("/lane_waypoints_array", 5, &RasCore::subTrajectoryCallback, this);
    sub_carla_actor_list = n.subscribe("/carla/actor_list", 1, &RasCore::subActorCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &RasCore::subOdomCallback, this);

    sub_carla_obj = n.subscribe("/carla/objects", 1, &RasCore::subObjCallback, this);
	sub_shift = n.subscribe("/feedback_info", 10, &RasCore::subShiftCallback, this);
	pub_obj = n.advertise<ras_carla::RasObjectArray>("/managed_objects", 5);
	// pub_erase = n.advertise<std_msgs::Int32>("/erase_signal", 1);

	m_obj_map.clear();
}


void RasCore::callbackDynamicReconfigure(ras_carla::rasConfig &config, uint32_t lebel)
{
	m_conservative_recognition = config.conservative?true:false;
	m_max_vision = config.max_vision_range;
	m_min_vision = config.min_vision_range;
	m_keep_time = config.keep_time;
	m_ego_name = config.ego_name;
}

// get ego_vehicle id to remove ego_object from obstacles
void RasCore::subActorCallback(const carla_msgs::CarlaActorList &in_actor_list)
{
    // ROS_INFO("subActorCallback");
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
    // ROS_INFO("subOdomCallback");
	float min_dist_power = 9.0, dist;
    int ego_wp = 0;

	m_ego_pose = in_odom.pose.pose;
	m_ego_twist = in_odom.twist.twist;
    // std::cout << (int)(100.0 / m_wp_interval) << std::endl;
	// find closest waypoint from m_waypoints
	for (size_t i = m_ego_wp; i < m_ego_wp + (int)(m_max_vision / m_wp_interval); i++)
	{
		dist = pow(m_ego_pose.position.x - m_wps_vec[i].position.x, 2) + pow(m_ego_pose.position.y - m_wps_vec[i].position.y, 2);
        // std::cout << dist << std::endl;
		if (min_dist_power > dist)
		{
			ego_wp = i;
			min_dist_power = dist;
		}
	}

    m_ego_wp = ego_wp;
    m_brakable_wp = m_ego_wp + (int)((pow(m_ego_twist.linear.x * 3.6, 2) / (254 * 0.7)) / m_wp_interval);
}


void RasCore::subTrajectoryCallback(const autoware_msgs::LaneArray &in_array)
{
    // ROS_INFO("subTrajectoryCallback");
	for (const auto &itr : in_array.lanes[0].waypoints)
	{
		m_wps_vec.emplace_back(itr.pose.pose);
	}
	m_wp_interval = sqrt(pow(m_wps_vec[0].position.x - m_wps_vec[1].position.x, 2) + pow(m_wps_vec[0].position.y - m_wps_vec[1].position.y, 2));
}


void RasCore::subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array)
{
    if (m_wps_vec.empty() || m_ego_wp == 0)
    {
        std::stringstream ss;
        ss << "waypoint or ego odometry is not subscrived yet, ego_wp : " <<  m_ego_wp;
        ROS_ERROR("waypoint or ego odometry is not subscrived yet");
        std::cout << m_ego_wp << std::endl;
        return;
    }

    // ROS_INFO("subObjCallback");
	geometry_msgs::Pose in_obj_pose;
    ras_carla::RasObject ras_obj;

	for (const auto &e : in_obj_array.objects)
	{
		ras_obj.object = e;
        // ROS_INFO("get pose from ego");
		in_obj_pose = Ras::tfTransformer(ras_obj.object.pose, ras_obj.object.header.frame_id, m_ego_name);
        // ROS_INFO("calc distance");
		ras_obj.distance = sqrt(pow(in_obj_pose.position.x, 2) + pow(in_obj_pose.position.y, 2));
		if (m_min_vision < ras_obj.distance && ras_obj.distance < m_max_vision && ras_obj.object.id != m_ego_id)
		{
			ras_carla::RasObject &selected_obj = m_obj_map[ras_obj.object.id];
			selected_obj.object = ras_obj.object;
			selected_obj.distance = ras_obj.distance;
			// selected_obj.is_front = (in_obj_pose.position.x > 0.5) ? true : false;
			// selected_obj.is_same_lane = (fabs(in_obj_pose.position.y) > 0.5) ? true : false;
            setCrossWp(selected_obj);
            std::cout << "map obj:" << e.id << "," << selected_obj.cross_wp_list[0] << std::endl;
		}
	}
	takeAttendance();
}


RasCore::setCrossWp(ras_carla::RasObject &in_obj)
{
    std::vector<int> cross_wp_list;

    switch(in_obj.object.classification)
    {
        case 4:
        {
            // ROS_INFO("setCrossWp pedestrian");
            std::vector<float> obj_closest_wp_vec;
            float min_dist_of_wp_obj = m_max_vision, dist_of_closestwp_obj, dist_of_verticalwp_obj, inner_prod, dist_of_wp_ego, obj_closestwp_vec_x, obj_closestwp_vec_y, obj_verticalwp_vec_x, obj_verticalwp_vec_y;
            int obj_wp, cross_wp;

            // find closest waypoint from object
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                dist_of_closestwp_obj = sqrt(pow(in_obj.object.pose.position.x - itr->position.x, 2) + pow(in_obj.object.pose.position.y - itr->position.y, 2));
                if (dist_of_closestwp_obj < min_dist_of_wp_obj)
                {
                    min_dist_of_wp_obj = dist_of_closestwp_obj;
                    obj_wp = std::distance(m_wps_vec.begin(), itr);
                }
            }

            obj_closestwp_vec_x = m_wps_vec[obj_wp].position.x - in_obj.object.pose.position.x;
            obj_closestwp_vec_y = m_wps_vec[obj_wp].position.y - in_obj.object.pose.position.y;

            // judge wheather the closest_wp should be considered
            dist_of_wp_ego = m_wp_interval * (obj_wp - m_ego_wp);
            if (dist_of_wp_ego > 0.0 && dist_of_closestwp_obj < dist_of_wp_ego && dist_of_closestwp_obj / in_obj.object.twist.linear.x < dist_of_wp_ego / m_ego_twist.linear.x)
            {
                std::cout << "pedestrian closest wp added"  << in_obj.object.id << " closest wp is :" << obj_wp << std::endl;
                cross_wp_list.emplace_back(obj_wp);
            }

            // // find vertical way of the object from the way to the closest waypoint
            // for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            // {
            //     cross_wp = std::distance(m_wps_vec.begin(), itr);
            //     obj_verticalwp_vec_x = itr->position.x - in_obj.object.pose.position.x;
            //     obj_verticalwp_vec_y = itr->position.y - in_obj.object.pose.position.y;
            //     dist_of_verticalwp_obj = sqrt(pow(obj_verticalwp_vec_x, 2) + pow(obj_verticalwp_vec_y, 2));
            //     inner_prod = obj_verticalwp_vec_x * obj_closestwp_vec_x + obj_verticalwp_vec_y * obj_closestwp_vec_y; // inner prod of closest_wp-in_obj vec and target_wp-in_obj vec
            //
            //     if (inner_prod > 0.1 * dist_of_verticalwp_obj * min_dist_of_wp_obj || itr == m_wps_vec.begin())
            //     {
            //         continue;
            //     }
            //     // inner prod of neibour waypoint vec and in_obj-target_wp vec
            //     inner_prod = (itr->position.x - (itr-1)->position.x) * obj_verticalwp_vec_x + (itr->position.y - (itr-1)->position.y) * obj_verticalwp_vec_y;
            //     // judge wheather the found other way cross the ego_path vertically
            //     if (inner_prod < 0.98 * dist_of_verticalwp_obj * m_wp_interval)
            //     {
            //         continue;
            //     }
            //     // judge wheather the wp should be considered
            //     dist_of_wp_ego = m_wp_interval * (cross_wp - m_ego_wp);
            //     if (dist_of_verticalwp_obj > 0.0 && dist_of_verticalwp_obj < dist_of_wp_ego && dist_of_verticalwp_obj / in_obj.object.twist.linear.x < dist_of_wp_ego / m_ego_twist.linear.x)
            //     {
            //         std::cout << "pedestrian cross wp added : " << in_obj.object.id << " cross_wp is :" << cross_wp << std::endl;
            //         cross_wp_list.emplace_back(cross_wp);
            //     }
            // }
        }

        case 6:
        {
            // ROS_INFO("setCrossWp car");
            float inner_prod, obj_vec_x, obj_vec_y, obj_closestwp_vec_x, obj_closestwp_vec_y, obj_vec_len, obj_wp_len;
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr ++)
            {
                obj_vec_len = pow(in_obj.object.twist.linear.x * 3.6, 2) / (254 * 0.7);
                obj_vec_x = obj_vec_len * cos(Ras::quatToYaw(in_obj.object.pose.orientation));
                obj_vec_y = obj_vec_len * sin(Ras::quatToYaw(in_obj.object.pose.orientation));
                obj_closestwp_vec_x = itr->position.x - in_obj.object.pose.position.x;
                obj_closestwp_vec_y = itr->position.y - in_obj.object.pose.position.y;
                obj_wp_len = sqrt(pow(obj_closestwp_vec_x, 2) + pow(obj_closestwp_vec_y, 2));

                inner_prod = obj_vec_x * obj_closestwp_vec_x + obj_vec_y * obj_closestwp_vec_y;
                if (inner_prod < obj_vec_len * obj_wp_len * 0.98)
                {
                    // std::cout << "car cross wp added"  << in_obj.object.id<< std::endl;
                    cross_wp_list.emplace_back(std::distance(m_wps_vec.begin(), itr));
                    break;
                }
            }
        }
    }
    std::sort(cross_wp_list.begin(), cross_wp_list.end());
    in_obj.cross_wp_list.emplace_back(cross_wp_list);
}


void RasCore::takeAttendance()
{
    // ROS_INFO("tekaAttendance");
	std::vector<int> erase_key_vec, critical_obj_id_vec;
	ras_carla::RasObjectArray obj_array;
    ras_carla::RasObject wall;
    int min_dist = m_wps_vec.size(), dist, closest_wp;

	for (const auto &e : m_obj_map)
	{
		// erace old object
        // ROS_INFO("get erase key");
		if ((ros::Time::now() - e.second.object.header.stamp) > ros::Duration(m_keep_time))
		{
            erase_key_vec.emplace_back(e.first);
            continue;
		}

        // get closest stop waypoint and effective obstacles
        for (size_t i = 0; i < e.second.cross_wp_list.size(); i++)
        {
            // std::cout << "cross wp list size : " << e.second.cross_wp_list.size() << "index : " << i << std::endl;
            dist = e.second.cross_wp_list[i] - m_ego_wp;
            // if closer waypoint is found, clear vector and insert new one
            if (0 < dist_of_wp_ego && dist < min_dist)
            {
                critical_obj_id_vec.clear();
                min_dist = dist;
                critical_obj_id_vec.emplace_back(e.second.object.id);
                closest_wp = e.second.cross_wp_list[i];
                // std::cout << e.first << ", " <<  closest_wp << std::endl;
            }
            // if critical obstacle is found at second time, add it to vector
            else if (dist == min_dist)
            {
                critical_obj_id_vec.emplace_back(e.first);
            }
        }
	}

    // erace old object
    for (const auto &e : erase_key_vec)
    {
        // ROS_INFO("eraced obj");
        m_obj_map.erase(e);
    }


    // judge wheather the object is critical or not and add to output list
    for (auto e : m_obj_map)
    {
        // ROS_INFO("find critical obj");
        e.second.is_interaction = (std::find(critical_obj_id_vec.begin(), critical_obj_id_vec.end(), e.second.object.id) != critical_obj_id_vec.end()) ? true : false;
        obj_array.objects.emplace_back(e.second);
    }

    // ROS_INFO("create wall obj");

    // finally add wall
    wall.object.header.stamp = ros::Time::now();
    wall.object.header.frame_id = "map";
    wall.object.id = 0;
    wall.object.pose = m_wps_vec[closest_wp];
    wall.object.shape.type = shape_msgs::SolidPrimitive::BOX;
    wall.object.shape.dimensions.emplace_back(0.1);
    wall.object.shape.dimensions.emplace_back(5.0);
    wall.object.shape.dimensions.emplace_back(2.0);
    wall.is_interaction = false;
    obj_array.objects.emplace_back(wall);

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
        obj.touch_num ++;
		takeAttendance();
	}
}
