#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
#include "std_msgs/Int32.h"


class PositionManager
{
private:
    ros::Publisher pub_obj;
    ros::Publisher pub_erase_sinal;
    ros::Subscriber sub_obj;
    ros::Subscriber sub_shift;

    tf::TransformListener tf_listener;

    const static int vector_size = 10;
    std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;

    int keep_time;
    // ros::Time last_pub_time;

    // uint32_t managed_id;
    //<id, index>
    std::map<uint32_t, int> id_index_map;


public:
    PositionManager();

private:
    void subObjCallback(const swipe_obstacles::detected_obstacle_array &in_msgs);
    geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id);
    void containerManage(const swipe_obstacles::detected_obstacle_array &in_msgs);
    void subShiftCallback(const swipe_obstacles::detected_obstacle &in_msg);
    void obstaclePublish();
};


PositionManager::PositionManager(): keep_time(2)
{
    ros::NodeHandle n;

    // pub_erase_sinal = n.advertise<std_msgs::Int32>("/swipe_erase_signal", 5);
    pub_obj = n.advertise<swipe_obstacles::detected_obstacle_array>("/managed_obstacles", 5);
    sub_obj = n.subscribe("/detected_obstacles", 5, &PositionManager::subObjCallback, this);
    sub_shift = n.subscribe("/shifted_info", 5, &PositionManager::subShiftCallback, this);
    // obstacle_vec.resize(vector_size);
}


void PositionManager::subObjCallback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    // ROS_INFO("manager Get obj info");
    // データリストに格納
    containerManage(in_msgs);
    obstaclePublish();
}


void PositionManager::containerManage(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    swipe_obstacles::detected_obstacle in_msg;
    std::vector<int> vec_active_index_list;
    std::vector<swipe_obstacles::detected_obstacle> new_obstacle_vec;
    std::map<uint32_t, int> new_id_index_map;

    float distance, min_distance;
    int flag=0, id_buf;
    size_t index;
    uint32_t inherit_id = 0;

    // new_obstacle_vec.resize(in_msgs.obstacles.size());

    // 入力された障害物に対してループ
    for(index = 0; index < in_msgs.obstacles.size(); index++)
    {
        in_msg = in_msgs.obstacles[index];
        in_msg.pose = tfTransformer(in_msg.pose, in_msg.header.frame_id, "map");
        min_distance = 4.0;

        // 現在格納されている障害物リストを探索し,入力と同一(とみなせる)障害物データのidを検索
        for(auto obstacle_vec_itr = obstacle_vec.begin(); obstacle_vec_itr != obstacle_vec.end(); obstacle_vec_itr++)
        {
            // idがすでに存在していたら
            if(obstacle_vec_itr->id == in_msg.id)
            {
                inherit_id = obstacle_vec_itr->id;
                break;
            }
            // else
            // {
            //     // 既に近距離に検出されていた場合も同一障害物とみなす
            //     distance = std::pow(obstacle_vec_itr->pose.position.x - in_msg.pose.position.x, 2) + std::pow(obstacle_vec_itr->pose.position.y - in_msg.pose.position.y, 2);
            //     if (distance < min_distance && )
            //     {
            //         min_distance = distance;
            //         inherit_id = obstacle_vec_itr->id;
            //     }
            // }
        }

        // 現在の障害物リストに同一の障害物がすでにあったら最新のidに更新,全く新しい障害物だったら検出時のidを採用.
        if (inherit_id != 0)
        {
            // in_msg.id = inherit_id; //軽くトラッキングする場合は必要?
            in_msg.shift_x = obstacle_vec.at(id_index_map.at(inherit_id)).shift_x;
            in_msg.shift_y = obstacle_vec.at(id_index_map.at(inherit_id)).shift_y;
        }

        std::cout << "inherit id:" << inherit_id << " in_msg id:" << in_msg.id << std::endl;
        std::cout << "shift_x : " << in_msg.shift_x << std::endl;
        // 近距離の障害物を同一とみなしてid付けする際に,入力障害物内でidがダブる可能性があるのでcheck
        if(!new_id_index_map.count(in_msg.id))
        {
            // 新しい障害物リストに追加
            new_obstacle_vec.emplace_back(in_msg);
            // 新しいindex検索辞書に追加, indexをそのまま用いると入力障害物がダブった際にずれる
            new_id_index_map.emplace(in_msg.id, new_obstacle_vec.size()-1);
            std::cout << "id:" << in_msg.id << " index:" << new_obstacle_vec.size()-1 << std::endl;
        }
    }

    //古い障害物リストから保持したいデータを抽出する
    for(auto obstacle_vec_itr = obstacle_vec.begin(); obstacle_vec_itr != obstacle_vec.end(); obstacle_vec_itr++)
    {
        // 新鮮なデータかつ新しい障害物リストに追加されていなかったら
        if((ros::Time::now() - obstacle_vec_itr->detected_time) < ros::Duration(keep_time) && !new_id_index_map.count(obstacle_vec_itr->id))
        {
            new_obstacle_vec.emplace_back(*obstacle_vec_itr);
            new_id_index_map.emplace(obstacle_vec_itr->id, new_obstacle_vec.size()-1);
            std::cout << "(extracted) id:" << obstacle_vec_itr->id << "index:" << new_obstacle_vec.size()-1 << std::endl;
            // index++;
        }
    }

    // 現在のデータリストを初期化
    obstacle_vec.clear();
    id_index_map.clear();

    // データリストを更新
    obstacle_vec.resize(new_obstacle_vec.size());
    obstacle_vec = new_obstacle_vec;
    id_index_map = new_id_index_map;

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
    // 取得障害物探索 obstacle_vecを上から検索
    // for(auto obstacle_vec_itr = obstacle_vec.begin(); obstacle_vec_itr != obstacle_vec.end(); obstacle_vec_itr++)
    // {
    //     for(size_t in_msg_index=0; in_msg_index<in_msgs.obstacles.size(); in_msg_index++)
    //     {
    //         in_msg = in_msgs.obstacles[in_msg_index];
    //         // in_msg.pose = tf_transformer(in_msgs.obstacles[i].pose, in_msgs.header.frame_id, "world");
    //
    //         // idがすでに存在していたら
    //         if(obstacle_vec_itr->id == in_msg.id)
    //         {
    //             obstacle_vec_itr->pose = in_msg.pose;
    //             obstacle_vec_itr->detected_time = in_msg.detected_time;
    //             obstacle_vec_itr->score = in_msg.score;
    //             vec_active_index_list.emplace_back(std::distance(obstacle_vec.begin(), obstacle_vec_itr));
    //             flag = 1;
    //             break;
    //         }
    //         else
    //         {
    //         // 既に近距離に検出されていた場合
    //             distance = std::pow(i->pose.position.x - in_msg.pose.position.x, 2) + std::pow(i->pose.position.y - in_msg.pose.position.y, 2);
    //             if(distance < 4.0)
    //             {
    //                 std::cout << "+" << in_msg.id << "-" << obstacle_vec_itr->id << std::endl;
    //                 // std::cout << id_dict << std::endl;
    //                 // std::cout << "new" << in_msg.id << "old" << i->id << std::endl;
    //                 // index = id_dict.at(i->id) % vector_size;
    //                 // real_id を再登録
    //                 // id_dict.emplace(in_msg.id, i->managed_id);
    //                 // id_dict.erase(i->id);
    //                 obstacle_vec_itr->id = in_msg.id;
    //                 obstacle_vec_itr->pose = in_msg.pose;
    //                 obstacle_vec_itr->detected_time = in_msg.detected_time;
    //                 obstacle_vec_itr->score = in_msg.score;
    //                 vec_active_index_list.emplace_back(std::distance(obstacle_vec.begin(), obstacle_vec_itr));
    //                 flag = 1;
    //                 break;
    //             }
    //         }
    //     }
    // }
    // // 新しい障害物なら
    // if (flag == 0)
    // {
    //     int avairable_index = vector_size+1;
    //
    //     for (int i=0; i < vector_size; i++)
    //     {
    //         auto obstacle_vec_itr = std::find(vec_active_index_list.begin(), vec_active_index_list.end(), i);
    //         if (itr == vec_active_index_list.end())
    //         {
    //             avairable_index = i;
    //             break;
    //         }
    //     }
    //
    //     if (avairable_index < vector_size)
    //     {
    //         obstacle_vec.at(avairable_index) = in_msg;
    //         obstacle_vec.at(avairable_index).managed_id = managed_id;
    //         managed_id++;
    //     }
    //     else
    //     {
    //         ROS_ERROR("full_of_container");
    //     }
    //     // std::cout << "+" << in_msg.id << std::endl;
    //     // id_dict.emplace(in_msg.id, managed_id);
    //     // index = managed_id % vector_size;
    // }

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
        //ros::Duration(1.0).sleep();
    }

    tf::poseMsgToTF(current_pose, current_tf);
    transformed_tf = transform * current_tf;
    tf::poseTFToMsg(transformed_tf, transformed_pose);

    return transformed_pose;
}


void PositionManager::obstaclePublish()
{
    swipe_obstacles::detected_obstacle_array out_msgs;
    std_msgs::Int32 erase_signal;
    int flag = 0;

    // for(std::vector<obstacle_info>::const_iterator i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
    // 障害物配列msgに追加していく
    for(auto i = obstacle_vec.begin(); i != obstacle_vec.end(); i++)
    {
        out_msgs.obstacles.emplace_back(*i);
        std::cout << "published id is " << i->id << std::endl;
        // ROS_INFO_STREAM(*i);
        flag = 1;
    }

    // 障害物配列メッセージに
    if(flag)
    {
        pub_obj.publish(out_msgs);
        std::cout << "managed info is published" << std::endl;
        // last_pub_time = ros::Time::now();
    }
    // else if(ros::Time::now() - last_pub_time > ros::Duration(keep_time))
    // {
    //         erase_signal.data = 1;
    //         pub_erase_sinal.publish(erase_signal);
    //         last_pub_time = ros::Time::now();
    // }
}


void PositionManager::subShiftCallback(const swipe_obstacles::detected_obstacle &in_msg)
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
// out_pose.pose = tf_transformer(camera_obj_pose);
