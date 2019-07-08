#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sound_play/sound_play.h>
#include <swipe_obstacles/closest_obstacle.h>

#include <cmath>

class YpTeleopStudy
{
    private :
        ros::Subscriber sub_joy;
        ros::Subscriber sub_twist;
        ros::Subscriber sub_closest_obstacle;
        ros::Publisher pub_yp_cmd;
        sound_play::SoundClient sc;


        geometry_msgs::Twist in_twist;
        geometry_msgs::Twist twist;
        float max_twist_speed;
        float accel;
        float brake;
        float pub_rate;

        ros::Timer timer;
		bool rosbag_flag ;
        bool mode;

        bool scenario_runner;
        swipe_obstacles::closest_obstacle closest_obstacle;
        ros::Time stop_time;

	public :
        YpTeleopStudy();

    private :
        void joyCallback(const sensor_msgs::Joy &in_msg);
        void twistCallback(const geometry_msgs::TwistStamped &in_msg);
        void timerCallback(const ros::TimerEvent&);
        void closestObstacleCallback(const swipe_obstacles::closest_obstacle &in_msg);
};


YpTeleopStudy::YpTeleopStudy(): mode(false), scenario_runner(false), rosbag_flag(0), pub_rate(0.1), max_twist_speed(1.0)
{
	ros::NodeHandle n;

    sub_joy = n.subscribe("/joy", 1, &YpTeleopStudy::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &YpTeleopStudy::twistCallback, this);
    sub_closest_obstacle = n.subscribe("/closest_obstacle", 5, &YpTeleopStudy::closestObstacleCallback, this);
    sub_waypoint_callback = n.subscribe("/closest_waypoint", 5, &YpTeleopStudy::waypointCallback, this);

    pub_yp_cmd = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &YpTeleopStudy::timerCallback, this);
}


void YpTeleopStudy::timerCallback(const ros::TimerEvent&)
{
    float speed_change;
    float current_twist_speed, aim_twist_speed;

    current_twist_speed = twist.linear.x;

    if(!accel && !brake)
    {
        aim_twist_speed = in_twist.linear.x;
    }
    else
    {
        aim_twist_speed = current_twist_speed
                        + accel * 0.25 * pub_rate * ((max_twist_speed - current_twist_speed) / max_twist_speed)
                        - brake * 0.5 * pub_rate * (0.1 + (max_twist_speed - current_twist_speed) / max_twist_speed);
    }

    // automode
    if(mode)
    {
        speed_change = aim_twist_speed - current_twist_speed;
        if(speed_change > 0.25 * pub_rate)
        {
            aim_twist_speed = current_twist_speed + 0.25 * pub_rate * ((max_twist_speed - current_twist_speed) / max_twist_speed);
        }
        if(speed_change < -0.5 * pub_rate)
        {
            aim_twist_speed = current_twist_speed - 0.5 * pub_rate * (0.1 + (max_twist_speed - current_twist_speed) / max_twist_speed);
        }
    }

    aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
    aim_twist_speed = (aim_twist_speed > 0.0) ? aim_twist_speed : 0.0;

    twist.linear.x = aim_twist_speed;
    twist.angular.z = in_twist.angular.z;
    // std::cout << "aim_twist=" << aim_twist_speed << std::endl;
    // std::cout << "current_twist=" << current_twist_speed << std::endl;
    // std::cout << "twist=" << twist.linear.x << std::endl;
    pub_yp_cmd.publish(twist);
}


void YpTeleopStudy::closestObstacleCallback(const swipe_obstacles::closest_obstacle &in_msg)
{
    closest_obstacle = in_msg;
}


void YpTeleopStudy::twistCallback(const geometry_msgs::TwistStamped &in_msg)
{
    if(mode)
    {
        in_twist.linear.x = in_msg.twist.linear.x;
        in_twist.angular.z = in_msg.twist.angular.z;

        if (closest_obstacle.brief_stop && closest_obstacle.distance < 5.0)
        {
            in_twist.linear.x = 0.0;

            if(stop_time == ros::Duration(0))
            {
                stop_time == ros::Time::now();
            }
            else
            {
                stop_time = ros::Time::now() - stop_time;
            }

        }
    }
}


void YpTeleopStudy::joyCallback(const sensor_msgs::Joy &in_msg)
{
    int dash = 0;
    float base_speed = 0.5;
    double sec_interval;

    if(in_msg.axes[5] == 1.0)
    {
        accel = 0.0;
    }else
    {
        accel = (1.0 - in_msg.axes[5]) * 0.5;
    }

    if(in_msg.axes[2] == 1.0)
    {
        brake = 0.0;
    }else
    {
        brake = (1.0 - in_msg.axes[2]) * 0.5;
    }

    //A button [0]
    if (in_msg.buttons[0])
    {
        if(scenario_runner)
        {
            sc.playWave("/usr/share/sounds/robot_sounds/mini_jump.wav");
        }
        else
        {
            sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
        }
    }
    // B button [1]
    if (in_msg.buttons[1])
    {
        if(scenario_runner)
        {
            scenario_runner = false;
        }
        else if(!scenario_runner)
        {
            scenario_runner = true;
            ROS_INFO("scenario mode\n");
        }
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
    // BACK [6]
    // Logicoool [8]
    // left joy click [9]
    // right joy click [10]

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
	YpTeleopStudy yp_teleop;

    ros::spin();
    return (0);
}
