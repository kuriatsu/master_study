#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sound_play/sound_play.h>
#include <cmath>

class YpTeleopStudy
{
    private :
        ros::Subscriber sub_joy;
        ros::Subscriber sub_twist;
        ros::Publisher pub_yp_cmd;

        sound_play::SoundClient sc;
        ros::Timer timer;

        geometry_msgs::Twist in_twist;
        geometry_msgs::Twist twist;
        float deceleration;
        float acceleration;
        float accel;
        float brake;
        float max_twist_speed;
        float current_twist_speed;
		bool rosbag_flag ;
        // ros::Time accel_start;
        // ros::Time brake_start;
        ros::Time last_joy_time;
        // mode=1: autoware mode=0:joystick
		bool mode;

	public :
        YpTeleopStudy();

    private :
        void joy_callback(const sensor_msgs::Joy &in_msg);
        void twist_callback(const geometry_msgs::TwistStamped &in_msg);
        void timer_callback(const ros::TimerEvent&);
};


YpTeleopStudy::YpTeleopStudy(): mode(false), rosbag_flag(0), acceleration(0.0), deceleration(0.0), max_twist_speed(1.0)
{
	ros::NodeHandle n;
    // twist = {};
    sub_joy = n.subscribe("/joy", 1, &YpTeleopStudy::joy_callback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &YpTeleopStudy::twist_callback, this);
    pub_yp_cmd = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);
    last_joy_time = ros::Time::now();
    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(0.1), &YpTeleopStudy::timer_callback, this);
}


void YpTeleopStudy::timer_callback(const ros::TimerEvent&)
{
    if(mode)
    {
        current_twist_speed = twist.linear.x;

        if(current_twist_speed < max_twist_speed){

            acceleration += accel * (max_twist_speed - current_twist_speed) * 0.025;
        }
        std::cout << "acceleration=" << acceleration << std::endl;

        if(current_twist_speed > 0.0){

            deceleration += brake * (max_twist_speed + 0.1 - current_twist_speed) * 0.05;
        }
        std::cout << "deceleration=" << deceleration << std::endl;

        if(current_twist_speed < deceleration){

            twist.linear.x = 0.0;
        }
        else{

            twist.linear.x = in_twist.linear.x + acceleration - deceleration;
        }

        if(twist.linear.x - current_twist_speed > 0.025 && current_twist_speed < max_twist_speed){

            twist.linear.x = current_twist_speed + (max_twist_speed - current_twist_speed) * 0.025;
        }

        if(current_twist_speed - twist.linear.x > 0.05 && current_twist_speed > 0.0)
        {
            twist.linear.x = current_twist_speed - (max_twist_speed + 0.1 - current_twist_speed) * 0.05;
            if(twist.linear.x < 0.0)
            {
                twist.linear.x = 0.0;
            }
        }
    }
    else
    {
        twist.linear.x = in_twist.linear.x;
    }

    twist.angular.z = in_twist.angular.z;
    std::cout << "twist=" << twist.linear.x << std::endl;
    pub_yp_cmd.publish(twist);
}


void YpTeleopStudy::twist_callback(const geometry_msgs::TwistStamped &in_msg)
{
    // max_twist_speed = ((max_twist_speed < in_msg.twist.linear.x) ? in_msg.twist.linear.x : max_twist_speed);

    if(mode)
    {
        in_twist.linear.x = in_msg.twist.linear.x;
        in_twist.angular.z = in_msg.twist.angular.z;
        // current_twist_speed = twist.linear.x;
    }
}


void YpTeleopStudy::joy_callback(const sensor_msgs::Joy &in_msg)
{
    int dash = 0;
    float base_speed = 0.5;
    double sec_interval;

    if(in_msg.axes[5] == 1.0)
    {
        accel = 0.0;
        acceleration = 0.0;
    }else
    {
        accel = (1.0 - in_msg.axes[5]) * 0.5;
    }

    if(in_msg.axes[2] == 1.0)
    {
        brake = 0.0;
        deceleration = 0.0;
    }else
    {
        brake = (1.0 - in_msg.axes[2]) * 0.5;
    }

    //A button [0]
    if (in_msg.buttons[0])
    {
    	sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
    }
    // X button [2]
    if (in_msg.buttons[2])
    {
        if(mode)
        {
            mode = false;
            sc.playWave("/usr/share/sounds/robot_sounds/pipe.wav");
            ROS_INFO("manual mode\n");
        }else if(!mode)
        {
            mode = true;
            sc.playWave("/usr/share/sounds/robot_sounds/powerup.wav");
            ROS_INFO("autonomous mode\n");
        }
    }
    // Y button [3]
    if (in_msg.buttons[3])
    {
         sc.playWave("/usr/share/sounds/robot_sounds/coin.wav");
    }
    // LB [4]
    if (in_msg.buttons[4])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
        dash += base_speed;
    }
    // RB [5]
    if (in_msg.buttons[5])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
        dash += base_speed;
    }

    // START [7]
    if (in_msg.buttons[7])
    {
        if(!rosbag_flag)
        {
            ROS_INFO("bag_record_on");
            sc.playWave("/usr/share/sounds/robot_sounds/new_world.wav");
            system("bash ~/ros/catkin_ws/src/teleop/src/bag_recorder.sh &");
            rosbag_flag = true;
        }else if(rosbag_flag)
        {
            ROS_INFO("bag_record_off");
            sc.playWave("/usr/share/sounds/robot_sounds/break_brick_block.wav");
            system("bash ~/ros/catkin_ws/src/teleop/src/bag_stopper.sh");
            rosbag_flag = false;
        }
    }
    // B button [1]
    // BACK [6]
    // Logicoool [8]

    if(!mode)
    {
        in_twist.linear.x =  dash + base_speed * in_msg.axes[4];
        in_twist.angular.z = 0.5 * in_msg.axes[0] * in_msg.axes[4];
        // current_twist_speed = twist.linear.x;
    }


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "yp_teleop_study0");
	ros::NodeHandle n;
	YpTeleopStudy ypteleop;

    ros::spin();
    return (0);
}
