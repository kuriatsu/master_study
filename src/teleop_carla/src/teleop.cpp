#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>

#include <dynamic_reconfigure/server.h>
#include <teleop_carla/teleopConfig.h>

class Teleop
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_odom;
    ros::Publisher pub_twist;

    float pub_rate;

    float throttle;
    float brake;
    float target_vel;
    float target_angular;
    float current_twist;
    // float max_radius;
    ros::Timer timer;

    // dynamic_reconfigure params
    float max_vel;
    float max_wheel_angle;
    float accel_step;
    float brake_step;
    float decel_step;
    int controller;
    bool autonomous_mode;
    bool rosbag_flag ;
    float back;

    dynamic_reconfigure::Server<teleop_carla::teleopConfig> server;
    dynamic_reconfigure::Server<teleop_carla::teleopConfig>::CallbackType server_callback;
public:
    Teleop();

private:
    void joyCallback(const sensor_msgs::Joy &in_joy);
    void twistCallback(const geometry_msgs::TwistStamped &in_twist);
    void odomCallback(const nav_msgs::Odometry &in_odom);
    void timerCallback(const ros::TimerEvent&);
    void callbackDynamicReconfigure(teleop_carla::teleopConfig &config, uint32_t lebel);
};


Teleop::Teleop(): throttle(0.0), brake(0.0), back(1.0), pub_rate(0.1)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&Teleop::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &Teleop::twistCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 1, &Teleop::odomCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist_cmd", 1);

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &Teleop::timerCallback, this);
}


void Teleop::callbackDynamicReconfigure(teleop_carla::teleopConfig &config, uint32_t lebel)
{
    max_vel = config.max_speed;
    max_wheel_angle = 90.0 - config.max_steer;
    accel_step = config.acceleration_coefficient * 9.8 * pub_rate * 3.6;
    brake_step = config.deceleration_coefficient * 9.8 * pub_rate  * 3.6;
    decel_step = 1.0 - config.deceleration_coefficient;
    autonomous_mode = config.autonomous_mode?true:false;
    rosbag_flag = config.rosbag_flag?true:false;
    controller = config.controller;
}

void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
    if (controller == 0)
    {
        throttle = (in_joy.axes[5] == -0.0) ? 0.0 : (1.0 - in_joy.axes[5]) * 0.5;
        brake = (in_joy.axes[2] == -0.0) ? 1.0 : (1.0 + in_joy.axes[2]) * 0.5;
        // std::cout << "accel: " << accel << " brake: " << brake << std::endl;

        if(in_joy.buttons[1])
        {
            back = (back == 1.0) ? -1.0 : 1.0;
        }

        if(in_joy.buttons[2])
        {
            autonomous_mode = !autonomous_mode;
        }

        // START [7]
        if (in_joy.buttons[7])
        {
            rosbag_flag = !rosbag_flag;

            if(!rosbag_flag)
            {
                ROS_INFO("bag_record_on");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            }else
            {
                ROS_INFO("bag_record_off");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            }
        }
    }
    else if(controller == 1)
    {
        throttle = (in_joy.axes[2] == 0.0) ? 0.0 : (1.0 + in_joy.axes[2]) * 0.5;
        brake = (in_joy.axes[3] == 0.0) ? 1.0 : (1.0 - in_joy.axes[3]) * 0.5;
        // vel_step = (out_twist.linear.x + vel_step < 0.1 && !back) ? 0.0;
        if(in_joy.buttons[3])
        {
            autonomous_mode = !autonomous_mode;
            back = 1.0;
        }

        if(in_joy.buttons[2] && !autonomous_mode)
        {
            back = (back == 1.0) ? -1.0 : 1.0;
        }

        //
        // if(in_joy.buttons[19])
        // {
        //     max_vel++;
        // }
        // if(in_joy.buttons[20])
        // {
        //     max_vel--;
        // }
        // START [7]
        if (in_joy.buttons[1])
        {
            rosbag_flag = !rosbag_flag;

            if(!rosbag_flag)
            {
                ROS_INFO("bag_record_on");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            }else
            {
                ROS_INFO("bag_record_off");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            }
        }
    }

    if (!autonomous_mode)
    {
        target_angular = - current_twist * tan((90.0 - max_wheel_angle) * in_joy.axes[0] * M_PI / 180) / 3.0;
    }
}


void Teleop::twistCallback(const geometry_msgs::TwistStamped &in_twist)
{
    if (autonomous_mode)
    {
        target_vel = in_twist.twist.linear.x;
        target_angular = -in_twist.twist.angular.z;
    }
}


void Teleop::odomCallback(const nav_msgs::Odometry &in_odom)
{
    current_twist = in_odom.twist.twist.linear.x;
}

void Teleop::timerCallback(const ros::TimerEvent&)
{
    geometry_msgs::Twist out_twist;
    float max_angular = fabs(current_twist) * tan(max_wheel_angle * M_PI / 180) / 3.0;
    target_vel = (fabs(target_vel) < (throttle * max_vel)) ? (throttle * max_vel) : target_vel;
    target_vel *= brake;
    std::cout << "current_twist: " << current_twist << " target_vel: " << target_vel << std::endl;

    float vel_diff = target_vel - fabs(current_twist);
    std::cout << "vel_Diff: " << vel_diff << std::endl;

    if (-brake_step > vel_diff || vel_diff > accel_step)
    {
        out_twist.linear.x = current_twist + ((vel_diff > 0.0) ? accel_step : -brake_step);
    }
    else
    {
        out_twist.linear.x = current_twist + vel_diff;
    }

    // std::cout << "target_angular" << target_angular << std::endl;
    out_twist.linear.x *= back;
    out_twist.angular.z = (fabs(target_angular) < max_angular) ? target_angular : (target_angular < 0.0) ? -max_angular : max_angular;
    // std::cout << "out_angular" << out_twist.angular.z << std::endl;

    pub_twist.publish(out_twist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    Teleop teleop;
    ros::spin();
    return(0);
}
