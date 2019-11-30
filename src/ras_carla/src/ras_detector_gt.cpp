#include <ros/ros.h>
#include "ras_detector_gt.h"




DetectorGT::DetectorGT()
{
	ros::NodeHandle n;

	pub_obj = n.advertise<ras_carla::RasObjectArray>("/detected_objects", 5);
	sub_carla_obj = n.subscribe("/carla/objects", 5, &DetectorGT::subObjCallback, this);

}


void DetectorGT::subObjCallback(const derived_object_msgs::ObjectArray &in_object_array)
{
	
	geometry_msgs::Pose ego_pose, object_pose; // obj_pose : position on [ego_vehicle] frame
	derived_object_msgs::Object in_object; // buffer for subscrived msgs

	// buffer for publish msg
	ras_carla::RasObject ras_object;
	ras_carla::RasObjectArray ras_object_array;

	for (size_t index=0; index < in_object_array.objects.size(); index++)
	{
	
		in_object = in_object_array.objects[index];

		// transform obj position to confirm that the object is in the specific area
		object_pose = tfTransformer(in_object.pose, in_object.header.frame_id, "ego_vehicle");

		// add information if object is in the specific area
		if (0.5 < object_pose.position.x && object_pose.position.x < 100)
		{
			if (-20 < object_pose.position.y && object_pose.position.y < 20)
			{
				// add object label depend of the classification number on carla
				switch(in_object.classification)
				{
					case 4:
						ras_object.label = "person";
						break;
					case 5:
						ras_object.label = "bicycle";
						break;
					case 6:
						ras_object.label = "car";
						break;
					case 7:
						ras_object.label = "car";
						break;
					case 8:
						ras_object.label = "car";
						break;
				}

				ras_object.header = in_object.header;
				ras_object.id = in_object.id;

				ras_object.score = in_object.classification_certainty;
				ras_object.distance = sqrt(pow(object_pose.position.x, 2) + pow(object_pose.position.y, 2));
				ras_object.pose = in_object.pose;
				ras_object.twist = in_object.twist;
				ras_object.accel = in_object.accel;

				ras_object_array.obstacles.push_back(ras_object);
				ROS_INFO("push_back_obstacle");
			}
		}
	}

	pub_obj.publish(ras_object_array);
}


geometry_msgs::Pose DetectorGT::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
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
