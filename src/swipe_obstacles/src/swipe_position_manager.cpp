#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

define
struct obstacle_info{

    uint32_t id;
    geometry_msgs::Pose pose;
    float shift;
}

class PositionManager{

    private:
        ros::Publisher pub_obj;
        ros::Subscriber sub_obj;
        tf::TransformListener tf_listener;
        std::vector<obstacle_info> obstacle_info_vec;
        const static int vector_size = 10;
        const static int keep_time = 10;

    public:
        PositionManager();

    private:
        void sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg);
        geometry_msgs::Pose tf_transformer(const geometry_msgs::Pose &in_pose, const std::string &in_frame_id);
        void store_obj(obstacle_info &in_msg);
        void obstacle_info_publish();
};


PositionManager::PositionManager()
{
    ros::NodeHandle n;

    pub_obj = n.advertise<swipe_obstacles::detected_obstacle_array>("/managed_obstacles", 5);
    sub_obj = n.subscribe("/detected_obstacles", 5, &PositionManager::sub_obj_callback, this);
    sub_shift = n.subscribe("/shifted_info", 5, &PositionManager::sub_shift_callback, this);

    obstacle_info_vec.resize(vector_size);
}


void PositionManager::obstacle_info_publish()
{
    swipe_obstacles::detected_obstacle_array out_msgs;

    // for(std::vector<obstacle_info>::const_iterator i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
    for(auto i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
    {
        // 古いデータはpublishしない.
        if((ros::Time(0) - i->detected_time) < ros::Duration(float(keep_time))){
            out_msgs.obstacles.push_back(*i);
        }
    }
    pub_obj.publish(out_msgs);
}


void PositionManager::sub_shift_callback(const swipe_obstacles::detected_obstacle &in_msg)
{
    obstacle_info_vec.at(in_msg.id % vector_size).shift = in_msg.shift;
    obstacle_info_publish();
}

void PositionManager::sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    obstacle_info out_obstacle_info;

    ROS_INFO("Get obj info");


    for(size_t i=0; i<msgs.obstacles.size(); i++)
    {
        out_obstacle_info.pose = tf_transformer(in_msgs.obstacles[i].pose, in_msgs.header.frame_id);
        out_obstacle_info.id = in_msgs.obstacles[i].id
        store_obj(out_obstacle_info);
    }
    obstacle_info_publish();
}


void PositionManager::store_obj(obstacle_info &in_msg)
{
    int flag=0, original_id;
    float disance, shift_buf;

    //データid=0または前データとidが同じだったらshiftを継承　.
    if(obstacle_info_vec.at[in_id%vector_size].id == 0 || obstacle_info_vec.at[in_id%10].id == in_id)
    {
        shift_buf = obstacle_info_vec.at[in_id%vector_size].shift;
        obstacle_info_vec.at[in_id%vector_size] = in_msg;
        obstacle_info_vec.at[in_id%vector_size].shift = shift_buf;
    }else
    {
        // 前データからidが変わっていたらとりあえず取得障害物の座標からの距離が2m以内の前データを見つける
        // for(std::vector<obstacle_info>::const_iterator i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
        for(auto i = obstacle_info_vec.begin(); i != obstacle_info_vec.end(); i++)
        {
            distance = (i->pose.position.x - in_msg.pose.position.x)**2 + (i->pose.position.y - in_msg.pose.position.y)**2;
            if(distance < 4.0)
            {
                // 見つけたら前データのshift値と元々のidを継承. 前データのいた場所のidは0にする.
                obstacle_info_vec.at[in_id%vector_size] = in_msg;
                obstacle_info_vec.at[in_id%vector_size].shift = i->shift;
                obstacle_info_vec.at[in_id%vector_size].original_id = i->original_id;
                i->id = 0;
                flag = 1;
                break;
            }
        }
        // 該当するものがなければそのまま挿入.
        if(flag == 0)
        {
            obstacle_info_vec.at[in_id%vector_size] = in_msg;
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
