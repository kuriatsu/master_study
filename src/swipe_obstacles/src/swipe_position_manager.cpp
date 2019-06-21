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
        tf::TransformListener tf_listener;
        std::vector<swipe_obstacles::detected_obstacle> obstacle_vec;
        const static int vector_size = 10;
        const static int keep_time = 3;
        uint32_t managed_id;
        //<real_id, managed_id>
        std::map<uint32_t, uint32_t> id_dict;

    public:
        PositionManager();

    private:
        void sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg);
        geometry_msgs::Pose tf_transformer(const geometry_msgs::Pose &in_pose, const std::string &in_frame_id);
        void store_obj(const swipe_obstacles::detected_obstacle *in_msg);
        void obstacle_publish();
};


PositionManager::PositionManager(): managed_id(0)
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

    // for(std::vector<obstacle_info>::const_iterator i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
    for(auto i = obstacle_vec.begin(); i != obstacle_vec.end(); i++)
    {
        // 古いデータはpublishしない.
        if((ros::Time(0) - i->detected_time) < ros::Duration(float(keep_time))){
            out_msgs.obstacles.push_back(*i);
        }
        else
        {
            id_dict.erase(i->id);
        }
    }
    pub_obj.publish(out_msgs);
}


void PositionManager::sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg)
{
    for (auto i=id_dict.begin(); i != id_dict.end(); i++)
    {
        if (i->second == in_msg.managed_id)
        {
            obstacle_vec.at(i->second).shift_x = in_msg.pose.position.x - obstacle_vec.at(i->second).pose.position.x;
            obstacle_vec.at(i->second).shift_y = in_msg.pose.position.y - obstacle_vec.at(i->second).pose.position.y;
            obstacle_vec.at(i->second).detected_time = ros::Time(0);
            obstacle_publish();
        }
    }
}


void PositionManager::sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    swipe_obstacles::detected_obstacle *out_msg;

    ROS_INFO("Get obj info");

    for(size_t i=0; i<msgs.obstacles.size(); i++)
    {
        out_msg = &in_msgs[i];
        out_msg->pose = tf_transformer(in_msgs.obstacles[i].pose, in_msgs.header.frame_id);
        store_obj(out_msg);
    }
    obstacle_publish();
}


void PositionManager::store_obj(const swipe_obstacles::detected_obstacle_array *in_msg)
{
    int flag=0, id_buf, index;
    float disance;

    // index = id_dict.at(in_msg.id) % vector_size;

    // idがすでにid_dictに存在していたら
    if (id_dict.count(in_msg->id))
    {
        index = id_dict.at(in_msg->id) % vector_size;

        obstacle_vec.at(index).pose = in_msg->pose;
        obstacle_vec.at(index).detected_time = in_msg->detected_time;
        obstacle_vec.at(index).score = in_msg->score;
    }
    else
    {
        for(auto i = obstacle_vec.begin(); i != obstacle_vec.end(); i++)
        {
            distance = (i->pose.position.x - in_msg->pose.position.x)**2 + (i->pose.position.y - in_msg->pose.position.y)**2;
            // 既に近距離に検出されていた場合
            if(distance < 4.0)
            {
                index = id_dict.at(i->id) % vector_size;
                // real_id を再登録
                in_dict.erase(i->id);
                id_dict.emplace(in_msg->id, i->managed_id);

                obstacle_vec.at(index).pose = in_msg->pose;
                obstacle_vec.at(index).detected_time = in_msg->detected_time;
                obstacle_vec.at(index).score = in_msg->detected_time;
                flag = 1;
                break;
            }
        }
        // 新しい障害物なら
        if (flag == 0)
        {
            id_dict.emplace(in_msg->id, managed_id);
            obstacle_vec.at[in_id%vector_size] = *in_msg;
            obstacle_vec.at[index].managed_id = managed_id;
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
		tf_listener.waitForTransform(in_frame_id, "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", in_frame_id, ros::Time(0), req_to_world);

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
