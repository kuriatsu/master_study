#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sound_play/sound_play.h>
#include "swipe_obstacles/closest_obstacle.h"
#include <dynamic_reconfigure/server.h>
#include <teleop_study/teleop_studyConfig.h>
#include "std_msgs/Int32.h"

#include <cmath>


class YpTeleopStudy
{
    private :
    ros::Subscriber sub_joy;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_closest_waypoint;
    // ros::Subscriber sub_closest_obstacle;
    ros::Publisher pub_yp_cmd;
    sound_play::SoundClient sc;


    geometry_msgs::Twist in_twist;
    geometry_msgs::Twist out_twist;
    int triger_frag;

    int start_waypoint;
    int final_waypoint;
    float max_twist_speed;
    float accel;
    float accel_limit;
    float brake;
    float brake_limit;
    float pub_rate;
    float dash = 0;
    float base_speed = 0.5;

    ros::Timer timer;
    bool rosbag_flag ;
    bool auto_mode;
    int closest_waypoint;

    dynamic_reconfigure::Server<teleop_study::teleop_studyConfig> server;
    dynamic_reconfigure::Server<teleop_study::teleop_studyConfig>::CallbackType server_callback;

    public :
    YpTeleopStudy();

    private :
    void joyCallback(const sensor_msgs::Joy &in_msg);
    void twistCallback(const geometry_msgs::TwistStamped &in_msg);
    void timerCallback(const ros::TimerEvent&);
    void closestWaypointCallback(const std_msgs::Int32 &in_msg);
    void dynamicCfgCallback(teleop_study::teleop_studyConfig &config, uint32_t level);
};


YpTeleopStudy::YpTeleopStudy(): auto_mode(false), triger_frag(0), rosbag_flag(0), accel(0.0), brake(1.0)
{
    ros::NodeHandle n;
    out_twist.linear.x = 0.0;
    out_twist.angular.z = 0.0;

    sub_joy = n.subscribe("/joy", 1, &YpTeleopStudy::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &YpTeleopStudy::twistCallback, this);
    sub_closest_waypoint = n.subscribe("/closest_waypoint", 1, &YpTeleopStudy::closestWaypointCallback, this);
    pub_yp_cmd = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);

    server_callback = boost::bind(&YpTeleopStudy::dynamicCfgCallback, this, _1, _2);
    server.setCallback(server_callback);

    n.getParam("autonomous_mode", auto_mode);

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &YpTeleopStudy::timerCallback, this);
}


void YpTeleopStudy::dynamicCfgCallback(teleop_study::teleop_studyConfig &config, uint32_t level)
{
    start_waypoint = std::stoi(config.start_waypoint);
    final_waypoint = std::stoi(config.final_waypoint);
    base_speed = config.base_speed;
    dash = config.dash;
    max_twist_speed = config.max_twist_speed;
    accel_limit = config.acceleration_limit;
    brake_limit = config.deceleration_limit;
    pub_rate = config.pub_rate;
}


void YpTeleopStudy::closestWaypointCallback(const std_msgs::Int32 &in_msg)
{
    closest_waypoint = in_msg.data;
}


void YpTeleopStudy::timerCallback(const ros::TimerEvent&)
{
    float speed_change;
    float current_twist_speed, aim_twist_speed;

    current_twist_speed = out_twist.linear.x;

    if (in_twist.linear.x >= 0.0)
    {
        aim_twist_speed = (in_twist.linear.x + accel) * brake;
    }else
    {
        aim_twist_speed = (in_twist.linear.x - accel) * brake;
    }

    aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
    aim_twist_speed = (aim_twist_speed > -max_twist_speed) ? aim_twist_speed : -max_twist_speed;

    // automode
    if(auto_mode)
    {
        speed_change = aim_twist_speed - current_twist_speed;
        if(speed_change > accel_limit * pub_rate)
        {
            aim_twist_speed = current_twist_speed + accel_limit * pub_rate * ((max_twist_speed - current_twist_speed) / max_twist_speed);
        }
        aim_twist_speed = (aim_twist_speed < max_twist_speed) ? aim_twist_speed : max_twist_speed;
        aim_twist_speed = (aim_twist_speed > 0.0) ? aim_twist_speed : 0.0;
    }

    out_twist.linear.x = aim_twist_speed;
    out_twist.angular.z = in_twist.angular.z;
    pub_yp_cmd.publish(out_twist);
}


void YpTeleopStudy::twistCallback(const geometry_msgs::TwistStamped &in_msg)
{
    std::cout << auto_mode << std::endl;
    if(auto_mode)
    {
        in_twist.linear.x = in_msg.twist.linear.x;
        in_twist.angular.z = in_msg.twist.angular.z;
        std::cout << in_twist.linear.x << std::endl;
        if (closest_waypoint > final_waypoint-5 /*&& in_twist.linear.x < base_speed*/)
        {
            in_twist.linear.x = max_twist_speed;
        }
    }
}


void YpTeleopStudy::joyCallback(const sensor_msgs::Joy &in_msg)
{
    double sec_interval;

    if(in_msg.axes[5] != -0.0) triger_frag = 1;
    if(in_msg.axes[2] != -0.0) triger_frag = 2;

    if(triger_frag < 1)
    {
        accel = 0.0;
    }else
    {
        accel = (1.0 - in_msg.axes[5]) * 0.5;
    }

    if(triger_frag < 2)
    {
        brake = 0.0;
    }else
    {
        brake = (1.0 + in_msg.axes[2]) * 0.5;
    }

    ROS_INFO("accel %f", accel);
    ROS_INFO("brake %f", brake);

    //A button [0]
    if (in_msg.buttons[0])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
    }
    // B button [1]

    // X button [2]
    if (in_msg.buttons[2])
    {
        if(auto_mode)
        {
            auto_mode = false;
            sc.playWave("/usr/share/sounds/robot_sounds/pipe.wav");
            ROS_INFO("manual mode\n");
        }else if(!auto_mode)
        {
            auto_mode = true;
            sc.playWave("/usr/share/sounds/robot_sounds/powerup.wav");
            ROS_INFO("autonomous mode\n");
        }
    }

    // Y button [3]
    if (in_msg.buttons[3])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/coin.wav");
    }

    if(!auto_mode)
    {
        in_twist.linear.x = base_speed * in_msg.axes[4];
        in_twist.angular.z = 0.5 * in_msg.axes[0] * in_msg.axes[4];
    }

    // LB [4]
    if (in_msg.buttons[4])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
        in_twist.linear.x += (in_twist.linear.x > 0.0) ? dash : -dash;
    }
    // RB [5]
    if (in_msg.buttons[5])
    {
        sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
        in_twist.linear.x += (in_twist.linear.x > 0.0) ? dash : -dash;
    }

    // START [7]
    if (in_msg.buttons[7])
    {
        if(!rosbag_flag)
        {
            ROS_INFO("bag_record_on");
            sc.playWave("/usr/share/sounds/robot_sounds/new_world.wav");
            system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            rosbag_flag = true;
        }
        else if(rosbag_flag)
        {
            ROS_INFO("bag_record_off");
            sc.playWave("/usr/share/sounds/robot_sounds/break_brick_block.wav");
            system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            rosbag_flag = false;
        }
    }
    // BACK [6]
    // Logicoool [8]
    // left joy click [9]
    // right joy click [10]
    ROS_INFO("joy_in_twist %f", in_twist.linear.x);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "yp_teleop_study1");
    YpTeleopStudy yp_teleop;

    ros::spin();
    return (0);
}
