# include <ros/ros.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/TwistStamped.h>
# include <tf/transform_broadcaster.h>


class carlaHelper
{
private:
	ros::Publisher pub_pose;
	ros::Publisher pub_estimate_twist;
	ros::Publisher pub_twist_carla;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_twist_cmd;

	geometry_msgs::PoseStamped ego_pose;

	tf::TransformBroadcaster br;
	ros::Timer pub_timer;

public:
	carlaHelper();

private:
	void subOdomCallback(const nav_msgs::Odometry &in_odom);
	void subTwistCmdCallback(const geometry_msgs::TwistStamped &in_twist);
	void pubTf(const ros::TimerEvent&);
};


carlaHelper::carlaHelper()
{
	ros::NodeHandle n;
	pub_pose = n.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1);
	pub_estimate_twist = n.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1);
	pub_twist_carla = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist_cmd", 1);

	sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 1, &carlaHelper::subOdomCallback, this);
	sub_twist_cmd = n.subscribe("/twist_cmd", 1, &carlaHelper::subTwistCmdCallback, this);

	ros::Duration(1).sleep();
	pub_timer = n.createTimer(ros::Duration(0.3), &carlaHelper::pubTf, this);
}


void carlaHelper::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	geometry_msgs::TwistStamped ego_twist;
	
	ego_pose.header.stamp = ros::Time::now();
	ego_pose.header.frame_id = "map";
	ego_twist.header.stamp = ros::Time::now();
	ego_twist.header.frame_id = "map";

	ego_pose.pose = in_odom.pose.pose;
	ego_twist.twist = in_odom.twist.twist;
	
	pub_estimate_twist.publish(ego_twist);
	pub_pose.publish(ego_pose);
	// pubTf(ego_pose.pose);

}

void carlaHelper::subTwistCmdCallback(const geometry_msgs::TwistStamped &in_twist)
{
	geometry_msgs::Twist out_twist;

	out_twist = in_twist.twist;
	out_twist.angular.z = -out_twist.angular.z;

	pub_twist_carla.publish(out_twist);
}


void carlaHelper::pubTf(const ros::TimerEvent&)
{
	tf::Transform transform;
	// transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
	// transform.setRotation(tf::quaternionMsgToTF(in_pose.orientation));
	tf::poseMsgToTF(ego_pose.pose, transform);
	// br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "map", "base_link"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "carla_helper_node");
	ROS_INFO("initializing");
	carlaHelper fake_ndt;
	ros::spin();
	return 0;
}