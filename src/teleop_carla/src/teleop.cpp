#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <cmath>

class Teleop
{
private:
	ros::Subscriber sub_joy;
	ros::Subscriber sub_twist;
	ros::Publisher pub_twist;

	geometry_msgs::Twist out_twist;
	float accel;
	float brake;
	float max_speed;
	float max_radius;
	bool autonomou_mode;
	bool rosbag_flag ;
	ros::Timer timer;

public:
	Teleop();

private:
	void joyCallback(const sensor_msgs::Joy &in_joy);
	void twistCallback(const geometry_msgs::TwistStamped &in_twist);
	void timerCallback(const ros::TimerEvent&);
};


Teleop::Teleop(): accel(0.0), brake(0.0), autonomou_mode(false), rosbag_flag(false), max_speed(60)
{
	ros::NodeHandle n;

	sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
	sub_twist = n.subscribe("/twist_cmd", 1, &Teleop::twistCallback, this);
	pub_twist = n.advertise("/carla/ego_vehicle/twist_cmd", 1);

	ros::Duration(1).sleep();
	timer - n.createTimer(ros::Duration(0.1), &Teleop::timerCallback, this);
}


void joyCallback(const sensor_msgs::Joy &in_joy)
{
	accel = (in_msg.axes[5] == -0.0) ? 0.0 : (1.0 - in_msg.axes[5]) * 0.5;
	brake = (in_msg.axes[5] == -0.0) ? 1.0 : (1.0 + in_msg.axes[2]) * 0.5;

	if(in_joy,button[2])
	{
		autonomous_mode = !autonomous_mode;
	}

	if (!autonomous_mode)
	{
		out_twist.linear.x = max_speed * accel * brake;
		out_twist.angular.z = 0.5 * out_twist.linear.x * in_msg.axes[4];
	}

	// START [7]
	if (in_msg.buttons[7])
	{
		rosbag_flag = !rosbag_flag;

		if(!rosbag_flag)
		{
			ROS_INFO("bag_record_on");
			system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
		}else
		{
			ROS_INFO("bag_record_off");
			system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
		}
	}
}


void twistCallback(const geometry_msgs::TwistStamped &in_twist)
{
	if (autonomou_mode)
	{
		out_twist = in_twist;
		out_twist.linear.x = (out_twist.linear.x > accel * max_speed) ? in_twist.linear.x : accel * max_speed;
		out_twist.linear.x = out_twist.linear.x * brake;
		out_twist.angular.z = -out_twist.angular.z;
	}
}


void timerCallback(const ros::TimerEvent&)
{
	float radius, steering_angle;
	float max_radius = 0.818300709; // tan(70deg)/wheelbase = 2.7474774194546 / 3.0

	if (fabs(out_twist.angular.z) > fabs(out_twist.linear.x / max_radius))
	{
		out_twist.angular.z = out_twist.linear.x / max_radius;
	}

	pub_twist.publish(out_twist);
}
max_angular_z = out_twist.linear.x * 2.7474774194546 / wheelbase

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
	ros::spin()
	return(0);
}
