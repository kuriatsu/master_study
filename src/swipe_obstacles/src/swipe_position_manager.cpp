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
        std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;
        const static int vector_size = 10;
        int keep_time;
        uint32_t managed_id;
        //<real_id, managed_id>
        std::map<uint32_t, uint32_t> id_dict;

    public:
        PositionManager();

    private:
        void sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg);
        geometry_msgs::Pose tf_transformer(const geometry_msgs::Pose &in_pose, const std::string &in_frame_id);
        void store_obj(const swipe_obstacles::detected_obstacle &in_msg);
        void obstacle_publish();
};


PositionManager::PositionManager(): managed_id(0), keep_time(2.0)
{
    ros::NodeHandle n;

    pub_obj = n.advertise<swipe_obstacles::detected_obstacle_array>("/managed_obstacles", 5);
    sub_obj = n.subscribe("/detected_obstacles", 5, &PositionManager::sub_obj_callback, this);
    sub_shift = n.subscribe("/shifted_info", 5, &PositionManager::sub_shift_callback, this);

    obstacle_vec.resize(vector_size);
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
            id_dict.erase(i->id);
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


void PositionManager::sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    swipe_obstacles::detected_obstacle out_msg;

    ROS_INFO("Get obj info");

    for(size_t i=0; i<in_msgs.obstacles.size(); i++)
    {
        out_msg = in_msgs.obstacles[i];
        out_msg.pose = tf_transformer(in_msgs.obstacles[i].pose, in_msgs.header.frame_id);
        store_obj(out_msg);
    }
    obstacle_publish();
}


void PositionManager::store_obj(const swipe_obstacles::detected_obstacle &in_msg)
{
    int flag=0, id_buf, index;
    float distance;

    // index = id_dict.at(in_msg.id) % vector_size;

    // idがすでにid_dictに存在していたら
    if (id_dict.count(in_msg.id))
    {
        ROS_INFO("pose updated");

        index = id_dict.at(in_msg.id) % vector_size;

        obstacle_vec.at(index).pose = in_msg.pose;
        obstacle_vec.at(index).detected_time = in_msg.detected_time;
        obstacle_vec.at(index).score = in_msg.score;
    }
    else
    {
        for(auto i = obstacle_vec.begin(); i != obstacle_vec.end(); i++)
        {
            distance = std::pow(i->pose.position.x - in_msg.pose.position.x, 2) + std::pow(i->pose.position.y - in_msg.pose.position.y, 2);
            // 既に近距離に検出されていた場合
            if(distance < 4.0)
            {
                std::cout << "+" << in_msg.id << "-" << i->id << std::endl;
                // std::cout << id_dict << std::endl;
                // std::cout << "new" << in_msg.id << "old" << i->id << std::endl;
                index = id_dict.at(i->id) % vector_size;
                // real_id を再登録
                id_dict.emplace(in_msg.id, i->managed_id);
                id_dict.erase(i->id);

                obstacle_vec.at(index).id = in_msg.id;
                obstacle_vec.at(index).pose = in_msg.pose;
                obstacle_vec.at(index).detected_time = in_msg.detected_time;
                obstacle_vec.at(index).score = in_msg.score;
                flag = 1;
                break;
            }
        }
        // 新しい障害物なら
        if (flag == 0)
        {
            std::cout << "+" << in_msg.id << std::endl;
            id_dict.emplace(in_msg.id, managed_id);
            index = managed_id % vector_size;
            obstacle_vec.at(index) = in_msg;
            obstacle_vec.at(index).managed_id = managed_id;
            managed_id++;
        }
    }
}


geometry_msgs::Pose PositionManager::tf_transformer(const geometry_msgs::Pose &in_pose, const std::string &in_frame_id)
{
    geometry_msgs::Pose world_pose;
    tf::Pose world_to_camera;
    tf::Pose req_to_camera;
    tf::StampedTransform req_to_world;

    try{
		tf_listener.waitForTransform(in_frame_id, "world", ros::Time::now(), ros::Duration(1.0));
		tf_listener.lookupTransform("world", in_frame_id, ros::Time::now(), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}

	tf::poseMsgToTF(in_pose, world_to_camera);
	req_to_camera = req_to_world * world_to_camera;
	tf::poseTFToMsg(req_to_camera, world_pose);

    return world_pose;
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
