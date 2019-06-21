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


class ObstacleVisualizer
{
	private:
		ros::Subscriber sub_obj;
		ros::Publisher pub_shift;
        std::vector<uint32_t> id_vec;

	public:
		ObstacleVisualizer();
		void sync_jsk_box();

	private:
		void sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void make_cube(const swipe_obstacles::detected_obstacle &in_msg);
        int id_vector_manager(const uint32_t &id);
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
};


ObstacleVisualizer::ObstacleVisualizer()
{
	ros::NodeHandle n;

	sub_detection = n.subscribe("/managed_obstacles", 5, &ObstacleVisualizer::sub_obstacles_callback, this);
	pub_shift = n.advertise<swipe_obstacles::detected_obstacle>("/shifted_info", 5);
}


void ObstacleVisualizer::sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    server->clear();

    for (size_t i=0; i< in_msgs.obstacles.size(); i++)
    {
        make_cube(in_msgs.obstacles[i]);
    }
	// for (size_t i=0; i< in_msgs.obstacles.size(); i++)
    // {
    //     auto itr = std::find(id_vec.begin(), id_vec.end(), in_msgs[i].managed_id);
    //
    //     if(itr != id_vec.end())
    //     {
    //         server->setPose(*itr, calc_boxpose(in_msgs.obstacles[i].pose, in_msgs.obstacles[i].shift));
    //     }
    //     else
    //     {
    //         make_cube(in_msgs.obstacles[i]);
    //         id_vec.push_back(in_msgs.obstacles[i].managed_id);
    //     }
	// }
    //
    // for (auto i=prev_obj_ids.begin(); i != prev_obj_ids.end(), i++)
    // {
    //     server->erase(*i);
    // }
    // // idデータリスト更新,新データリストは空にシておく.
    // prev_obj_ids = current_obj_ids;
    server->applyChanges();

}


void ObstacleVisualizer::make_cube(const swipe_obstacles::detected_obstacle &in_msg)
{
    swipe_obstacles::detected_obstacle out_obstacle_pose;
    std::stringstream ss;
    ss << in_msg.managed_id;

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = ss.str();
	int_marker.scale = 1.0;
    int_marker.pose = in_msg.pose;
    int_marker.pose.position.x = in_msg.pose.position.x + in_msg.shift_x;
	int_marker.pose.position.y = in_msg.pose.position.y + in_msg.shift_y;

	make_box_control(int_marker);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&ObstacleVisualizer::shift_feedback, this, _1));
}


visualization_msgs::InteractiveMarkerControl& ObstacleVisualizer::make_box_control(visualization_msgs::InteractiveMarker &msg)
{
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
	marker.scale.z = msg.scale*1.7;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 0.5;

	control.markers.push_back(marker);
	msg.controls.push_back(control);

	return msg.controls.back();
}


void ObstacleVisualizer::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    swipe_obstacles::detected_obstacle feedback_obstacle;
    std::istringstream sis;

    sis = std::istringstream(feedback->marker_name);
    feedback_obstacle.pose = feedback->pose;
    feedback_obstacle.managed_id << sis;
    pub_shift.publish()
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "swipe_obstacle_visualizar_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("swipe_obstacle_visualizar_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	ObstacleVisualizer obstacle_visualizar;
	ROS_INFO("Ready...");
	// server->applyChanges();

	ros::spin();
	server.reset();

	return 0;
}
