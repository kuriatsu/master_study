#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>

#include <cmath>


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
		void sub_obj_callback(const swipe_obstacles::detected_obstacle_array &in_msgs);
        void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void make_cube(const swipe_obstacles::detected_obstacle &in_msg);
        int id_vector_manager(const uint32_t &id);
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
};


class put_aster_obstacle{

	private:
		ros::Publisher pub_pointcloud;
		geometry_msgs::Pose obstacle_pose;
		pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
		ros::Timer timer;
		tf::TransformListener tf_listener;


	public :
		put_aster_obstacle();
		void sync_jsk_box();
		void get_obstacle_pose();

	private :
		void make_cube();
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		void pub_pointcloud_Nhz(const ros::TimerEvent&);
};


ObstacleVisualizer::ObstacleVisualizer()
{
	ros::NodeHandle n;

	sub_detection = n.subscribe("/managed_obstacles", 5, &ObstacleVisualizer::sub_obstacles_callback, this);
    pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/int_pointcloud", 1);

    	get_obstacle_pose();

    	make_cube();
    	ros::Duration(0.5).sleep();
    	timer = n.createTimer(ros::Duration(0.2), &put_aster_obstacle::pub_pointcloud_Nhz, this);


    }
}

void put_aster_obstacle::sync_jsk_box(){

	double roll, pitch, yaw;
	tf::Quaternion tf_quat(obstacle_pose.orientation.x, obstacle_pose.orientation.y, obstacle_pose.orientation.z, obstacle_pose.orientation.w);
	tf::Matrix3x3(tf_quat).getRPY( roll, pitch, yaw);

	out_cloud.width = 20;
	out_cloud.height = 5;
	out_cloud.points.resize(100);

	double x, y;
	unsigned int count = 0;

	for (unsigned int row = 0; row < 5; row++){
		for (unsigned int col = 0; col < 20; col++){

			pcl::PointXYZRGB &point = out_cloud.points[count];
			x = -0.5 + row * 0.2;
			y = -3.0 + col * 0.3;
			point.x = x*cos(2*M_PI+yaw) - y*sin(2*M_PI+yaw) + obstacle_pose.position.x;
			point.y = x*sin(2*M_PI+yaw) + y*cos(2*M_PI+yaw) + obstacle_pose.position.y ;
			point.z = obstacle_pose.position.z;
			point.r = point.g = point.b = 0.3;

			count++;
		}
	}

}


void put_aster_obstacle::pub_pointcloud_Nhz(const ros::TimerEvent&){

	sensor_msgs::PointCloud2 in_scan, out_scan;

	geometry_msgs::Pose velodyne_frame_pose;
	// geometry_msgs::Pose &world_frame_pose = out_cloud;

	pcl::toROSMsg(out_cloud, in_scan);
	in_scan.header.stamp = ros::Time::now();

	in_scan.header.frame_id = "world";
	out_scan.header.frame_id = "velodyne";

	if(!out_cloud.points.empty()){

		try{
			tf_listener.waitForTransform("world", "velodyne", in_scan.header.stamp, ros::Duration(0.2));

		}catch(...){
			ROS_INFO("world to velodyne transform ERROR");
		}

		pcl_ros::transformPointCloud("velodyne", in_scan, out_scan, tf_listener);

		out_scan.header.stamp = in_scan.header.stamp;
		pub_pointcloud.publish(out_scan);
		std::cout << "published" <<std::endl;
	}


}
void ObstacleVisualizer::sub_obstacles_callback(const swipe_obstacles::detected_obstacle_array &in_msgs)
{
    server->clear();

    for (size_t i=0; i< in_msgs.obstacles.size(); i++)
    {
        make_cube(in_msgs.obstacles[i]);
    }

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
