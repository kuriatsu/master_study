#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore(): max_recognize_distance(50.0), min_recognize_distance(3.0), keep_time(2.0), min_recognize_vel(1.0)
{
	ros::NodeHandle n;

    sub_carla_obj = n.subscribe("/carla/objects", 1, &RasCore::subObjCallback, this);
    sub_carla_actor_list = n.subscribe("/carla/actor_list", 1, &RasCore::subActorCallback, this);
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
    float closest_obj_dist = max_recognize_distance;
    ras_carla::RasObject *closest_obj;
    // std::cout << "subscribed objects" << std::endl;
    for (size_t index=0; index < in_obj_array.objects.size(); index++)
    {
      ras_obj.object = in_obj_array.objects[index];

      obj_pose = Ras::tfTransformer(ras_obj.object.pose, ras_obj.object.header.frame_id, "base_link");
      ras_obj.distance = sqrt(pow(obj_pose.position.x, 2) + pow(obj_pose.position.y, 2));

        if (min_recognize_distance < ras_obj.distance && ras_obj.distance < max_recognize_distance && ras_obj.object.id != ego_id)
        {

            ras_carla::RasObject &selected_obj = obj_map[ras_obj.object.id];
            selected_obj.object = ras_obj.object;
            selected_obj.distance = ras_obj.distance;
            selected_obj.is_front = (obj_pose.position.x > 0.5) ? true : false;
            selected_obj.is_just_back = (fabs(obj_pose.position.y) > 0.5) ? true : false;

            if (selected_obj.is_front)
            {
                if (ras_obj.distance < closest_obj_dist)
                {
                    closest_obj = &obj_map[ras_obj.object.id];
                    closest_obj_dist = ras_obj.distance;
                }
                if (ras_obj.distance < max_recognize_distance * 0.5)
                {
                    selected_obj.importance = 0.5;
                }

            }
            // std::cout << ras_obj.object.id << " added" << std::endl;
        }
    }
    closest_obj->importance = 1.0;
    containerManage();
    // std::cout << obj_map.size() << std::endl;
}


void RasCore::subActorCallback(const carla_msgs::CarlaActorList &in_actor_list)
{
    for (size_t index = 0; index < in_actor_list.actors.size(); index ++)
    {
        if (in_actor_list.actors[index].rolename == "ego_vehicle")
        {
            ego_id = in_actor_list.actors[index].id;
        }
    }
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
        // std::cout << itr->first;
      if ((ros::Time::now() - itr->second.object.header.stamp) < ros::Duration(keep_time))
      {
        calcDimension(itr->second);
        obj_array.objects.push_back(itr->second);
            // std::cout << " is left" << std::endl;
        }
        else
        {
            erase_key_vec.push_back(itr->first);
                // std::cout <<" is erased" << std::endl;
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
    float movable_dist, movable_vel, safety_dist;
    float clipped_ego_vel = ego_twist.linear.x > min_recognize_vel ? ego_twist.linear.x : min_recognize_vel;
    if(ego_twist.linear.x > min_recognize_vel)
    {

        switch(in_obj.object.classification)
        {
            case 4: //pedestrian
            // movable_vel = in_obj.object.twist.linear.x > min_recognize_vel ? in_obj.object.twist.linear.x : min_recognize_vel;
            // movable_dist = movable_vel * in_obj.distance / clipped_ego_vel;
            safety_dist = pow((ego_twist.linear.x+in_obj.object.twist.linear.x) * 3.6, 2) / (254 * 0.7);
            in_obj.object.shape.type = 1;
            in_obj.object.shape.dimensions[0] = safety_dist;
            in_obj.object.shape.dimensions[1] = safety_dist;
            break;

            case 6: //car
            {
                float inner_prod = cos(Ras::quatToYaw(ego_pose.orientation) - Ras::quatToYaw(in_obj.object.pose.orientation));
                in_obj.object.shape.type = 1;

                if (in_obj.is_front && inner_prod < 0 && in_obj.object.twist.linear.x > 0.1)
                {
                    // movable_vel = in_obj.object.twist.linear.x;
                    // movable_dist = movable_vel * in_obj.distance / clipped_ego_vel;
                    safety_dist = pow((ego_twist.linear.x+in_obj.object.twist.linear.x) * 3.6, 2) / (254 * 0.7);
                    in_obj.object.shape.dimensions[0] = safety_dist;
                    in_obj.object.pose.position.x += 0.5 * safety_dist * cos(Ras::quatToYaw(in_obj.object.pose.orientation));
                    in_obj.object.pose.position.y += 0.5 * safety_dist * sin(Ras::quatToYaw(in_obj.object.pose.orientation));
                }

                else if (!in_obj.is_front && !in_obj.is_just_back && inner_prod > 0 && in_obj.object.twist.linear.x - ego_twist.linear.x > 5.0)
                {
                    movable_vel = in_obj.object.twist.linear.x;
                    movable_dist = movable_vel * in_obj.distance / (in_obj.object.twist.linear.x - ego_twist.linear.x) ;
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
