#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <math.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
#include "swipe_obstacles/closest_obstacle.h"

class SwipeDetectorCarla
{
private:
	ros::Publisher pub_obstacle;
	ros::Subscriber sub_carla_obj;

	geometry_msgs::Pose ego_pose;
    tf::TransformListener tf_listener;

public:
	SwipeDetectorCarla();

private:
	void subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array);
	geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id);
};


SwipeDetectorCarla::SwipeDetectorCarla()
{
	ros::NodeHandle n;

	pub_obstacle = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
	sub_carla_obj = n.subscribe("/carla/objects", 5, &SwipeDetectorCarla::subObjCallback, this);

}


void SwipeDetectorCarla::subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array)
{
	

	geometry_msgs::Pose ego_pose, obj_pose; // obj_pose : position on [ego_vehicle] frame
	derived_object_msgs::Object in_obj; // buffer for subscrived msgs

	// buffer for publish msg
	swipe_obstacles::detected_obstacle detected_obstacle;
	swipe_obstacles::detected_obstacle_array detected_obstacle_array;

	for (size_t index=0; index < in_obj_array.objects.size(); index++)
	{
	
		in_obj = in_obj_array.objects[index];

		// transform obj position to confirm that the object is in the specific area
		obj_pose = tfTransformer(in_obj.pose, in_obj.header.frame_id, "ego_vehicle");

		// add object label depend of the classification number on carla
		switch(in_obj.classification)
		{
			case 4:
				detected_obstacle.label = "person";
				break;
			case 5:
				detected_obstacle.label = "bicycle";
				break;
			case 6:
				detected_obstacle.label = "car";
				break;
			case 7:
				detected_obstacle.label = "car";
				break;
			case 8:
				detected_obstacle.label = "car";
				break;
		}

		// add information if object is in the specific area
		if (0.5 < obj_pose.position.x && obj_pose.position.x < 100)
		{
			if (-20 < obj_pose.position.y && obj_pose.position.y < 20)
			{
				detected_obstacle.header = in_obj.header;
				detected_obstacle.id = in_obj.id;
				detected_obstacle.pose = in_obj.pose;
				detected_obstacle.score = in_obj.classification_certainty;
				detected_obstacle.distance = sqrt(pow(obj_pose.position.x, 2) + pow(obj_pose.position.y, 2));
				detected_obstacle_array.obstacles.push_back(detected_obstacle);
				ROS_INFO("push_back_obstacle");
			}
		}
	}

	pub_obstacle.publish(detected_obstacle_array);
}


geometry_msgs::Pose SwipeDetectorCarla::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
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



int main(int argc, char **argv)
{
	ros::init(argc, argv, "swipe_detector_carla_node");

	ROS_INFO("initialized detector");
	SwipeDetectorCarla swipe_detector_carla;

	ros::spin();
	return 0;

}