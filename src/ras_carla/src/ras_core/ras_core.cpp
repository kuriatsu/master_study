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
    pub_wp_obj = n.advertise<geometry_msgs::PointStamped>("/obj_wp", 5);
    pub_wp_cross = n.advertise<geometry_msgs::PointStamped>("/crossed_wp", 5);
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
    // for (size_t i = m_ego_wp; i < m_ego_wp + (int)(m_max_vision / m_wp_interval); i++)
	for (size_t i = 0; i < m_wps_vec.size(); i++)
	{
		dist = pow(m_ego_pose.position.x - m_wps_vec[i].position.x, 2) + pow(m_ego_pose.position.y - m_wps_vec[i].position.y, 2);
        // std::cout << i << "," << dist << std::endl;
		if (min_dist_power > dist)
		{
			ego_wp = i;
			min_dist_power = dist;
		}
	}

    m_ego_wp = ego_wp;
    m_brakable_wp = ego_wp + (int)((pow(m_ego_twist.linear.x * 3.6, 2) / (254 * 0.7)) / m_wp_interval);
    for (const auto &e : m_wp_obj_map[ego_wp])
    {
        ROS_INFO_STREAM(m_obj_map[e]);
        std::cout << "on : "<< e  << std::endl;
        m_obj_map[e].is_touched = false;
        std::cout << "to"<< std::endl;
        ROS_INFO_STREAM(m_obj_map[e]);
    }
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
        ROS_ERROR("waypoint or ego odometry is not subscrived yet");
        std::cout << m_ego_wp << std::endl;
        return;
    }

    m_wp_obj_map.clear();

    // ROS_INFO("subObjCallback");
	geometry_msgs::Pose in_obj_pose;
    ras_carla::RasObject ras_obj;
    std::vector<int> wp_vec;

	for (const auto &e : in_obj_array.objects)
	{
        if (e.pose.position.x == 0.0 && e.pose.position.y == 0.0 && e.pose.position.z == 0.0) continue;

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
            selected_obj.is_interaction = true;
            selected_obj.is_important = false;
			// selected_obj.is_front = (in_obj_pose.position.x > 0.5) ? true : false;
			// selected_obj.is_same_lane = (fabs(in_obj_pose.position.y) > 0.5) ? true : false;
            calcOccupancyWp(findWpOfObj(selected_obj), selected_obj);

            // std::cout << "map obj:" << e.id << "," << selected_obj.cross_wp_list[0] << std::endl;
		}
	}
	manageMarkers();
}


std::vector<int> RasCore::findWpOfObj(ras_carla::RasObject &in_obj)
{
    std::vector<int> wp_vec;

    switch(in_obj.object.classification)
    {
        case 4:
        {
            // ROS_INFO("findWpOfObj pedestrian");
            float min_dist_of_wp_obj = m_max_vision, inner_prod, obj_vec_x, obj_vec_y, obj_vec_len, obj_wp_vec_x, obj_wp_vec_y, obj_wp_vec_len, dist_of_wp_obj, wp_vec_x, wp_vec_y;
            int obj_wp, cross_wp;

            // find closest waypoint from object
            for (auto itr = m_wps_vec.begin(); itr < m_wps_vec.end(); itr++)
            // for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                dist_of_wp_obj = Ras::calcDistOfPoints(itr->position, in_obj.object.pose.position);
                if (dist_of_wp_obj < min_dist_of_wp_obj)
                {
                    min_dist_of_wp_obj = dist_of_wp_obj;
                    obj_wp = std::distance(m_wps_vec.begin(), itr);
                }
            }
            // std::cout << "obj_id : " << in_obj.object.id << " wp : " << obj_wp << "close" << std::endl;
            wp_vec.emplace_back(obj_wp);
            pubOccupancyWp(m_wps_vec[obj_wp].position, 0);

            // find vertical way of the object from the way to the closest waypoint
            obj_vec_x = m_wps_vec[obj_wp].position.x - in_obj.object.pose.position.x;
            obj_vec_y = m_wps_vec[obj_wp].position.y - in_obj.object.pose.position.y;

            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                cross_wp = std::distance(m_wps_vec.begin(), itr);
                if(cross_wp == 0) continue;
                if(cross_wp == obj_wp) continue;
                obj_wp_vec_x = itr->position.x - in_obj.object.pose.position.x;
                obj_wp_vec_y = itr->position.y - in_obj.object.pose.position.y;

                inner_prod = obj_vec_x * obj_wp_vec_x + obj_vec_y * obj_wp_vec_y; // inner prod of closest_wp-in_obj vec and target_wp-in_obj vec
                obj_vec_len = sqrt(pow(obj_vec_x, 2) + pow(obj_vec_y, 2));
                obj_wp_vec_len = sqrt(pow(obj_wp_vec_x, 2) + pow(obj_wp_vec_y, 2));
                if (fabs(inner_prod) < obj_vec_len * obj_wp_vec_len * 0.05 || inner_prod > obj_vec_len * obj_wp_vec_len * 0.8)
                {
                    wp_vec_x = itr->position.x - (itr-1)->position.x;
                    wp_vec_y = itr->position.y - (itr-1)->position.y;
                    inner_prod = wp_vec_x * obj_wp_vec_x + wp_vec_y * obj_wp_vec_y;
                    if (fabs(inner_prod) < m_wp_interval * sqrt(pow(obj_wp_vec_x, 2) + pow(obj_wp_vec_y, 2)) * 0.05)
                    {
                        // std::cout << "obj_id : " << in_obj.object.id << " wp : " << cross_wp << "cross" << std::endl;
                        wp_vec.emplace_back(cross_wp);
                        pubOccupancyWp(m_wps_vec[cross_wp].position, 1);
                    }
                }
            }
            break;
        }

        case 6:
        {
            // ROS_INFO("findWpOfObj car");
            float obj_vec_x, obj_vec_y, obj_wp_vec_x, obj_wp_vec_y, inner_prod, dist_of_wp_obj;
            obj_vec_x = cos(Ras::quatToYaw(in_obj.object.pose.orientation));
            obj_vec_y = sin(Ras::quatToYaw(in_obj.object.pose.orientation));

            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr ++)
            {
                obj_wp_vec_x = itr->position.x - in_obj.object.pose.position.x;
                obj_wp_vec_y = itr->position.y - in_obj.object.pose.position.y;

                inner_prod = obj_vec_x * obj_wp_vec_x + obj_vec_y * obj_wp_vec_y;
                dist_of_wp_obj = sqrt(pow(obj_wp_vec_x, 2) + pow(obj_wp_vec_y, 2));
                if (inner_prod > 1.0 * dist_of_wp_obj * 0.99)
                {
                    wp_vec.emplace_back(std::distance(m_wps_vec.begin(), itr));
                    pubOccupancyWp(m_wps_vec[std::distance(m_wps_vec.begin(), itr)].position, 1);
                    break;
                }
            }
            break;
        }
    }
    return wp_vec;
}


bool RasCore::isCollideObstacle(const ras_carla::RasObject &in_obj, const int &wp)
{
    float dist_of_wp_ego, dist_of_wp_obj;
    dist_of_wp_ego = (wp - m_ego_wp) * m_wp_interval;
    dist_of_wp_obj = Ras::calcDistOfPoints(in_obj.object.pose.position, m_wps_vec[wp].position);

    if (wp > m_ego_wp + m_max_vision / m_wp_interval || in_obj.is_touched)
        return false;

    switch (in_obj.object.classification)
    {
        case 4:
            return (dist_of_wp_obj < dist_of_wp_ego || dist_of_wp_obj / in_obj.object.twist.linear.x < dist_of_wp_ego / m_ego_twist.linear.x);
            break;
        case 6:
            return (dist_of_wp_ego > 0 && dist_of_wp_obj / in_obj.object.twist.linear.x < dist_of_wp_ego / m_ego_twist.linear.x);
            break;
    }
}


void RasCore::calcOccupancyWp(const std::vector<int> &in_wp_vec, const ras_carla::RasObject &in_obj)
{
    for (const auto &e : in_wp_vec)
    {
        m_wp_obj_map[e].emplace_back(in_obj.object.id);
    }
}


int RasCore::findWallWp(std::vector<int> &critical_obj_id_vec)
{
    for (const auto &e : m_wp_obj_map)
    {
        if (e.first < m_brakable_wp) continue;
        // std::cout << e.first << std::endl;
        critical_obj_id_vec.clear();

        for (const auto &obj_id : e.second)
        {
            if (isCollideObstacle(m_obj_map[obj_id], e.first))
            {
                critical_obj_id_vec.emplace_back(obj_id);
                // wall_wp = e.first;
                std::cout << "wall wp is :" << e.first << std::endl;
                return(e.first);
            }
        }
    }
    return 0;
}

void RasCore::manageMarkers()
{
    // ROS_INFO("tekaAttendance");
	std::vector<int> erase_key_vec, critical_obj_id_vec;
	ras_carla::RasObjectArray obj_array;
    ras_carla::RasObject wall;
    int wall_wp;

    wall_wp = findWallWp(critical_obj_id_vec);
    std::cout << "critical object : " << critical_obj_id_vec.size() << std::endl;

	for (auto &e : m_obj_map)
	{
		// erace old object
		if ((ros::Time::now() - e.second.object.header.stamp) > ros::Duration(m_keep_time))
		{
            erase_key_vec.emplace_back(e.first);
            continue;
		}

        if (std::find(critical_obj_id_vec.begin(), critical_obj_id_vec.end(), e.first) != critical_obj_id_vec.end())
        {
            e.second.is_important = true;
            e.second.is_interaction = true;
        }
        obj_array.objects.emplace_back(e.second);
	}

    // erace old object
    for (const auto &e : erase_key_vec)
    {
        m_obj_map.erase(e);
    }
    std::cout << wall_wp << std::endl;
    if (wall_wp != 0)
    {
        // finally add wall
        wall.object.header.stamp = ros::Time::now();
        wall.object.header.frame_id = "map";
        wall.object.id = 0;
        wall.object.pose = m_wps_vec[wall_wp];
        wall.object.shape.type = shape_msgs::SolidPrimitive::BOX;
        wall.object.shape.dimensions.emplace_back(0.1);
        wall.object.shape.dimensions.emplace_back(5.0);
        wall.object.shape.dimensions.emplace_back(2.0);
        wall.is_interaction = false;
        wall.is_important = true;
        obj_array.objects.emplace_back(wall);
    }

    // publish
	obj_array.header.stamp = ros::Time::now();
	obj_array.header.frame_id = "map";
	pub_obj.publish(obj_array);
}


void RasCore::pubOccupancyWp(const geometry_msgs::Point &in_pose, const int &type)
{
    geometry_msgs::PointStamped point;
    point.point = in_pose;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";

    switch (type)
    {
        case 0:
            pub_wp_obj.publish(point);
            break;
        case 1:
            pub_wp_cross.publish(point);
            break;
    }
}

void RasCore::subShiftCallback(const ras_carla::RasObject &in_msg)
{
	int id = in_msg.object.id;
	if (m_obj_map.find(id) != m_obj_map.end())
	{
        ras_carla::RasObject &obj = m_obj_map[id];
		std::cout << "touched id : " << std::endl;
        if (obj.is_important) obj.is_touched = true;
        else obj.is_touched = false;
		// manageMarkers();
	}
}
