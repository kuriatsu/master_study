#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

// struct obstacle_info
// {
//     uint32_t id;
//     uint32_t managed_id;
//     geometry_msgs::Pose pose;
//     float shift_x;
//     float shift_y;
// }

class PositionManager
{
private:
    ros::Publisher pub_obj;
    ros::Subscriber sub_obj;
    ros::Subscriber sub_shift;

    tf::TransformListener tf_listener;

    const static int vector_size = 10;
    std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;

    int keep_time;
    // uint32_t managed_id;
    //<real_id, stored_index>
    std::map<uint32_t, uint32_t> id_index_map;


public:
    PositionManager();

private:
    void subObjCallback(const swipe_obstacles::detected_obstacle_array &in_msgs);
    geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id);
    void containerManage(const swipe_obstacles::detected_obstacle_array &in_msgs);
    void sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg);
    void obstacle_publish();
};


PositionManager::PositionManager(): managed_id(1), keep_time(2.0)
{
    ros::NodeHandle n;

    pub_obj = n.advertise<swipe_obstacles::detected_obstacle_array>("/managed_obstacles", 5);
    sub_obj = n.subscribe("/detected_obstacles", 5, &PositionManager::sub_obj_callback, this);
    sub_shift = n.subscribe("/shifted_info", 5, &PositionManager::sub_shift_callback, this);

    obstacle_vec.resize(vector_size);
}


void PositionManager::subObjCallback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{

    // ROS_INFO("Get obj info");
    containerManage(in_msgs);
    obstacle_publish();
}


void PositionManager::containerManage(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    swipe_obstacles::detected_obstacle in_msg;
    int flag=0, id_buf, index;
    //<stored_index, flag>
    std::vector<int> container_active_index_list;

    float distance;

    // index = id_dict.at(in_msg.id) % vector_size;

    // if (id_index_map.count(in_msg.id))
    // {
    //     ROS_INFO("pose updated");
    //
    //     index = id_dict.at(in_msg.id) % vector_size;
    //
    // }
    // else
    // {
    for(auto container_itr = obstacle_vec.begin(); container_itr != obstacle_vec.end(); container_itr++)
    {
        // 取得障害物探索
        for(size_t obstacle_index=0; obstacle_index<in_msgs.obstacles.size(); i++)
        {
            in_msg = in_msgs.obstacles[obstacle_index];
            // in_msg.pose = tf_transformer(in_msgs.obstacles[i].pose, in_msgs.header.frame_id, "world");

            // idがすでに存在していたら
            if(container_itr->id == in_msg.id)
            {
                container_itr->pose = in_msg.pose;
                container_itr->detected_time = in_msg.detected_time;
                container_itr->score = in_msg.score;
                container_active_index_list.push_back(std::distance(obstacle_vec.begin(), container_itr));
                flag = 1;
                break;
            }
            else
            {
            // 既に近距離に検出されていた場合
                distance = std::pow(i->pose.position.x - in_msg.pose.position.x, 2) + std::pow(i->pose.position.y - in_msg.pose.position.y, 2);
                if(distance < 4.0)
                {
                    std::cout << "+" << in_msg.id << "-" << container_itr->id << std::endl;
                    // std::cout << id_dict << std::endl;
                    // std::cout << "new" << in_msg.id << "old" << i->id << std::endl;
                    // index = id_dict.at(i->id) % vector_size;
                    // real_id を再登録
                    // id_dict.emplace(in_msg.id, i->managed_id);
                    // id_dict.erase(i->id);
                    container_itr->id = in_msg.id;
                    container_itr->pose = in_msg.pose;
                    container_itr->detected_time = in_msg.detected_time;
                    container_itr->score = in_msg.score;
                    container_active_index_list.push_back(std::distance(obstacle_vec.begin(), container_itr));
                    flag = 1;
                    break;
                }
            }
        }
    }
    // 新しい障害物なら
    if (flag == 0)
    {
        int avairable_index = vector_size+1;

        for (int i=0; i < vector_size; i++)
        {
            auto container_itr = std::find(container_active_index_list.begin(), container_active_index_list.end(), i);
            if (itr == container_active_index_list.end())
            {
                avairable_index = i;
                break;
            }
        }

        if (avairable_index < vector_size)
        {
            obstacle_vec.at(avairable_index) = in_msg;
            obstacle_vec.at(avairable_index).managed_id = managed_id;
            managed_id++;
        }
        else
        {
            ROS_ERROR("full_of_container");
        }
        // std::cout << "+" << in_msg.id << std::endl;
        // id_dict.emplace(in_msg.id, managed_id);
        // index = managed_id % vector_size;
    }

}


geometry_msgs::Pose PositionManager::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
    geometry_msgs::Pose transformed_pose;
    tf::Pose current_tf;
    tf::Pose transformed_tf;
    tf::StampedTransform transform;

    try{
        tf_listener.waitForTransform(current_frame_id, target_frame_id, ros::Time::now(), ros::Duration(1.0));
        tf_listener.lookupTransform(target_frame_id, current_frame_id, ros::Time::now(), transform);

    }catch(...){
        ROS_INFO("velodyne to world transform ERROR");
    }

    tf::poseMsgToTF(current_pose, current_tf);
    transformed_tf = current_tf * transform;
    tf::poseTFToMsg(transformed_tf, transformed_pose);

    return transformed_pose;
}





void PositionManager::obstacle_publish()
{
    swipe_obstacles::detected_obstacle_array out_msgs;
    int flag = 0;

    // for(std::vector<obstacle_info>::const_iterator i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
    for(auto i = obstacle_vec.begin(); i != obstacle_vec.end(); i++)
    {
        // 古いデータはpublishしない.
        if((ros::Time::now() - i->detected_time) < ros::Duration(keep_time)){
            out_msgs.obstacles.push_back(*i);
            ROS_INFO_STREAM(*i);
            flag = 1;
        }
        else
        {
            std::cout << "-" << i->id << std::endl;
            // id_dict.erase(i->id);
        }
    }
    if(flag)
    {
        pub_obj.publish(out_msgs);
        ROS_INFO("published");
    }
}


void PositionManager::sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg)
{
    for (auto i=id_dict.begin(); i != id_dict.end(); i++)
    {
        if (i->second == in_msg.managed_id)
        {
            obstacle_vec.at(i->second).shift_x = in_msg.pose.position.x - obstacle_vec.at(i->second).pose.position.x;
            obstacle_vec.at(i->second).shift_y = in_msg.pose.position.y - obstacle_vec.at(i->second).pose.position.y;
            obstacle_vec.at(i->second).detected_time = ros::Time::now();
            obstacle_publish();
        }
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
// out_pose.pose = tf_transformer(camera_obj_pose);
