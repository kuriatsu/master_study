#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

namespace Ras{

geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
    static tf::TransformListener tf_listener;

    tf::Pose current_tf;
    tf::StampedTransform transform;
    tf::Pose transformed_tf;
    geometry_msgs::Pose transformed_pose;
    geometry_msgs::PoseStamped transform_origin;

    try{
        tf_listener.waitForTransform(current_frame_id, target_frame_id,  ros::Time(0), ros::Duration(1.0));
        // current_frame_id　から　target_frame_id　への座標変換
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

}
