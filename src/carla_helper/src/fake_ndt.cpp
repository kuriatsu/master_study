# include <ros/ros.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/TwistStamped.h>
# include <tf/transform_broadcaster.h>


class fakeNdt
{
private:
	ros::Publisher pub_twist;
	ros::Publisher pub_pose;
	ros::Subscriber sub_odom;

	geometry_msgs::Pose ego_pose;
	tf::TransformBroadcaster br;

public:
	fakeNdt();

private:
	void subOdomCallback(const nav_msgs::Odometry &in_odom);
	void pub_tf(const geometry_msgs::Pose &in_pose);
};


fakeNdt::fakeNdt()
{
	ros::NodeHandle n;
	pub_pose = n.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 5);
	pub_twist = n.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 5);

	sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &fakeNdt::subOdomCallback, this);
}


void fakeNdt::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	geometry_msgs::PoseStamped out_pose;
	geometry_msgs::TwistStamped out_twist;
	
	out_pose.header.stamp = ros::Time::now();
	out_pose.header.frame_id = "map";
	out_twist.header.stamp = ros::Time::now();
	out_twist.header.frame_id = "map";

	out_pose.pose = in_odom.pose.pose;
	out_twist.twist = in_odom.twist.twist;
	
	pub_twist.publish(out_twist);
	pub_pose.publish(out_pose);
	pub_tf(out_pose.pose);

}


void fakeNdt::pub_tf(const geometry_msgs::Pose &in_pose)
{
	tf::Transform transform;
	// transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
	// transform.setRotation(tf::quaternionMsgToTF(in_pose.orientation));
	tf::poseMsgToTF(in_pose, transform);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ego_vehicle"));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_ndt_node");
	ROS_INFO("initializing");
	fakeNdt fake_ndt;
	ros::spin();
	return 0;
}