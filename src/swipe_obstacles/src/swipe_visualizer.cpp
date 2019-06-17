#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"

#include <interactive_markers/interactive_marker_server.h>

#include <cmath>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


class ObstacleVisualizer{

	private:
		ros::Subscriber sub_obj;
		ros::Publisher pub_shift;
        std::vector<uint32_t> prev_obj_ids;
        std::vector<uint32_t> current_obj_ids;


	public:
		ObstacleVisualizer();
		void sync_jsk_box();

	private:
		void sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void make_cube();
        int id_vector_manager(const uint32_t &id);
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void calc_boxpose();
};


ObstacleVisualizer::ObstacleVisualizer()
{
	ros::NodeHandle n;

	sub_detection = n.subscribe("/managed_obstacles", 5, &ObstacleVisualizer::sub_obstacles_callback, this);
	pub_shift = n.advertise<swipe_obstacles::detected_obstacle>("/shifted_info", 5);
	//pub_jsk_box = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/int_boundingbox", 5);

	// out_jsk_msgs.dimensions.x = 1.0;
	// out_jsk_msgs.dimensions.y = 6.0;
	// out_jsk_msgs.dimensions.z = 2.0;
	// out_jsk_msgs.value = 1;
}


void ObstacleVisualizer::sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
	for (size_t i=0; i< in_msgs.obstacles.size(); i++)
    {
        if(id_vector_manager(in_msgs.obstacles[i].managed_id))
        {
            server->setPose(s, calc_boxpose(in_msgs.obstacles[i].pose, in_msgs.obstacles[i].shift));
        }
        else
        {
            make_cube(in_msgs.obstacles[i]);
        }
		// in_jsk_msgs = msgs->boxes[i];
		// out_jsk_msgs.header = msgs->boxes[i].header;
		// out_jsk_msgs.label = 1;
	}

    for (auto i=prev_obj_ids.begin(); i != prev_obj_ids.end(), i++)
    {
        server->erase(*i);
    }
    // idデータリスト更新,新データリストは空にシておく.
    prev_obj_ids = current_obj_ids;
    current_obj_ids.clear();
}

// prev_obj_idsでidが見つかったら削除return 1,見つからなくてもcurrent_obj_idsには追加 return 0
int ObstacleVisualizer::id_vector_manager(const uint32_t &id)
{
    auto itr = std::find(prev_obj_ids.begin(), prev_obj_ids.end(), id);
    current_obj_ids.push_back(id);

    if (itr != prev_obj_ids.end())
    {
        prev_obj_ids.erase(itr);
        return 1;
    }
    return 0;
}


////////////////////////////
void ObstacleVisualizer::make_cube()
{

	calc_boxpose();

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = "No.1";
	int_marker.scale = 1.0;
	int_marker.pose = out_jsk_msgs.pose;

	make_box_control(int_marker);
	sync_jsk_box();

	server->insert(int_marker);

	server->setCallback(int_marker.name, boost::bind(&ObstacleVisualizer::shift_feedback, this, _1));
	server->applyChanges();
}


void ObstacleVisualizer::calc_boxpose()
{
	geometry_msgs::Pose box_pose;
	tf::Pose world_to_velodyne;
	tf::Pose req_to_velodyne;
	tf::StampedTransform req_to_world;
	float theta = 0;
	if (in_jsk_msgs.pose.position.x != 0){
		theta = std::atan(out_jsk_msgs.pose.position.y / out_jsk_msgs.pose.position.x);;
	}

	box_pose.position.x = in_jsk_msgs.pose.position.x + shift * std::sin(theta);
	box_pose.position.y = in_jsk_msgs.pose.position.y + shift * std::cos(theta);
	box_pose.position.z = in_jsk_msgs.pose.position.z;
	box_pose.orientation.x = 0;
	box_pose.orientation.y = 0;
	box_pose.orientation.z = 0;
	box_pose.orientation.w = 1;

	try{
		tf_listener.waitForTransform("velodyne", "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", "velodyne", ros::Time(0), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}
	tf::poseMsgToTF(box_pose, world_to_velodyne);
	req_to_velodyne = req_to_world * world_to_velodyne;
	tf::poseTFToMsg(req_to_velodyne, out_jsk_msgs.pose);

	//ROS_INFO("x:%03f -> %03f, y:%03f -> %03f", velodyne_pose.position.x, world_pose.position.x, velodyne_pose.position.y, world_pose.position.y);
}


visualization_msgs::InteractiveMarkerControl& ObstacleVisualizer::make_box_control( visualization_msgs::InteractiveMarker &msg){

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.orientation.w = 1;

	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = msg.scale;
	marker.scale.y = msg.scale*6;
	marker.scale.z = msg.scale*3;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 0.5;

	control.markers.push_back(marker);
	msg.controls.push_back(control);

	return msg.controls.back();
}


void ObstacleVisualizer::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

	float permanet_shift;
	permanet_shift = std::sqrt(std::pow(feedback->pose.position.x - out_jsk_msgs.pose.position.x, 2.0) + std::pow(feedback->pose.position.y - out_jsk_msgs.pose.position.y, 2.0));
	if ((feedback->pose.position.y - out_jsk_msgs.pose.position.y) < 0){
		permanet_shift = -permanet_shift;
	}
	shift += permanet_shift;
	calc_boxpose();
	ROS_INFO_STREAM(shift);
}






int main(int argc, char **argv){

	ros::init(argc, argv, "swipe_obstacle_visualizar_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("swipe_obstacle_visualizar_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	ObstacleVisualizer obstacle_visualizar;
	ROS_INFO("Ready...");
	//server->applyChanges();

	ros::spin();
	server.reset();

	return 0;

}
