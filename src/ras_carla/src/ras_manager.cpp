#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"
#include "std_msgs/Int32.h"


class PositionManager
{
private:
    ros::Publisher pub_object;
    ros::Publisher pub_erase_sinal;
    ros::Subscriber sub_object;
    ros::Subscriber sub_shift;

    tf::TransformListener tf_listener;

    const static int vector_size = 10;
    std::vector<ras_carla::RasObject> obstacle_vec;

    int keep_time;
    std::map<uint32_t, int> id_index_map;


public:
    PositionManager();

private:
    void subObjCallback(const ras_carla::RasObjectArray &in_msgs);
    geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id);
    void containerManage(const ras_carla::RasObjectArray &in_msgs);
    void subShiftCallback(const ras_carla::RasObject &in_msg);
    void obstaclePublish();
};


PositionManager::PositionManager(): keep_time(2)
{
    ros::NodeHandle n;

    pub_object = n.advertise<ras_carla::RasObjectArray>("/managed_objects", 5);
    sub_object = n.subscribe("/detected_objects", 5, &PositionManager::subObjCallback, this);
    sub_shift = n.subscribe("/shifted_info", 5, &PositionManager::subShiftCallback, this);
}


void PositionManager::subObjCallback(const ras_carla::RasObjectArray &in_msgs)
{
    ROS_INFO("manager Get obj info");
    containerManage(in_msgs);
    obstaclePublish();
}


void PositionManager::containerManage(const ras_carla::RasObjectArray &in_msgs)
{
    ras_carla::RasObject in_msg;
    std::vector<int> vec_active_index_list;
    std::vector<ras_carla::RasObject> new_obstacle_vec;
    std::map<uint32_t, int> new_id_index_map;

    float distance, min_distance;
    int flag=0, id_buf;
    size_t index;
    uint32_t inherit_id = 0;

    for(index = 0; index < in_msgs.obstacles.size(); index++)
    {
        in_msg = in_msgs.obstacles[index];
        in_msg.pose = tfTransformer(in_msg.pose, in_msg.header.frame_id, "map");
        min_distance = 4.0;

        for(auto obstacle_vec_itr = obstacle_vec.begin(); obstacle_vec_itr != obstacle_vec.end(); obstacle_vec_itr++)
        {
            if(obstacle_vec_itr->id == in_msg.id)
            {
                inherit_id = obstacle_vec_itr->id;
                break;
            }
        }

        if (inherit_id != 0)
        {
            in_msg.shift_x = obstacle_vec.at(id_index_map.at(inherit_id)).shift_x;
            in_msg.shift_y = obstacle_vec.at(id_index_map.at(inherit_id)).shift_y;
        }

        if(!new_id_index_map.count(in_msg.id))
        {
            new_obstacle_vec.emplace_back(in_msg);
            new_id_index_map.emplace(in_msg.id, new_obstacle_vec.size()-1);
        }
    }

    for(auto obstacle_vec_itr = obstacle_vec.begin(); obstacle_vec_itr != obstacle_vec.end(); obstacle_vec_itr++)
    {
        if((ros::Time::now() - obstacle_vec_itr->detected_time) < ros::Duration(keep_time) && !new_id_index_map.count(obstacle_vec_itr->id))
        {
            new_obstacle_vec.emplace_back(*obstacle_vec_itr);
            new_id_index_map.emplace(obstacle_vec_itr->id, new_obstacle_vec.size()-1);
        }
    }

    obstacle_vec.clear();
    id_index_map.clear();

    obstacle_vec.resize(new_obstacle_vec.size());
    obstacle_vec = new_obstacle_vec;
    id_index_map = new_id_index_map;

}


geometry_msgs::Pose PositionManager::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
    geometry_msgs::Pose transformed_pose;
    tf::Pose current_tf;
    tf::Pose transformed_tf;
    tf::StampedTransform transform;

    try{
        tf_listener.waitForTransform(current_frame_id, target_frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), transform);

    }catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
    }

    tf::poseMsgToTF(current_pose, current_tf);
    transformed_tf = transform * current_tf;
    tf::poseTFToMsg(transformed_tf, transformed_pose);

    return transformed_pose;
}


void PositionManager::obstaclePublish()
{
    ras_carla::RasObjectArray out_msgs;
    std_msgs::Int32 erase_signal;
    int flag = 0;

    for(auto i = obstacle_vec.begin(); i != obstacle_vec.end(); i++)
    {
        out_msgs.obstacles.emplace_back(*i);
        flag = 1;
    }

    if(flag)
    {
        pub_object.publish(out_msgs);
    }
}


void PositionManager::subShiftCallback(const ras_carla::RasObject &in_msg)
{
    if (id_index_map.count(in_msg.id))
    {
        obstacle_vec.at(id_index_map.at(in_msg.id)).shift_x = in_msg.pose.position.x - obstacle_vec.at(id_index_map.at(in_msg.id)).pose.position.x;
        obstacle_vec.at(id_index_map.at(in_msg.id)).shift_y = in_msg.pose.position.y - obstacle_vec.at(id_index_map.at(in_msg.id)).pose.position.y;
        obstacle_vec.at(id_index_map.at(in_msg.id)).detected_time = ros::Time::now();
        obstaclePublish();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_position_manager_node");

    ROS_INFO("Initialized");

    PositionManager position_manager;

    ros::spin();
    return 0;
}
