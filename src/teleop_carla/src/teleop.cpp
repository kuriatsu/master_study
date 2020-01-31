#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <cmath>

#include <dynamic_reconfigure/server.h>
#include <teleop_carla/teleopConfig.h>

class Teleop
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_twist;
    ros::Publisher pub_twist;

    float pub_rate;

    geometry_msgs::Twist out_twist;
    float accel;
    float brake;
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
    void timerCallback(const ros::TimerEvent&);
    void callbackDynamicReconfigure(teleop_carla::teleopConfig &config, uint32_t lebel);
};


Teleop::Teleop(): accel(0.0), brake(0.0), back(1.0), pub_rate(0.1)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&Teleop::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &Teleop::twistCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist_cmd", 1);

    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &Teleop::timerCallback, this);
}


void Teleop::callbackDynamicReconfigure(teleop_carla::teleopConfig &config, uint32_t lebel)
{
    max_vel = config.max_speed;
    max_wheel_angle = 90.0 - config.max_steer;
    accel_step = config.acceleration_coefficient * 9.8 * pub_rate;
    brake_step = config.deceleration_coefficient * 9.8 * pub_rate;
    decel_step = 1.0 - config.deceleration_coefficient;
    autonomous_mode = config.autonomous_mode?true:false;
    rosbag_flag = config.rosbag_flag?true:false;
    controller = config.controller;
}

void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
    if (controller == 0)
    {
        accel = (in_joy.axes[5] == -0.0) ? 0.0 : (1.0 - in_joy.axes[5]) * 0.5 * accel_step * 3.6;
        brake = (in_joy.axes[2] == -0.0) ? 1.0 : (1.0 - in_joy.axes[2]) * 0.5 * brake_step * 3.6;
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
        accel = (in_joy.axes[2] == 0.0) ? 0.0 : (1.0 + in_joy.axes[2]) * 0.5 * accel_step;
        brake = (in_joy.axes[3] == 0.0) ? 1.0 : (1.0 + in_joy.axes[3]) * 0.5 * brake_step;
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
        out_twist.angular.z = - out_twist.linear.x * tan((90 - max_wheel_angle) * in_joy.axes[0] * M_PI / 180) / 3.0;
    }
}


void Teleop::twistCallback(const geometry_msgs::TwistStamped &in_twist)
{
    if (autonomous_mode)
    {
        out_twist = in_twist.twist;
        out_twist.angular.z = -out_twist.angular.z;
    }
}


void Teleop::timerCallback(const ros::TimerEvent&)
{
    std::cout << "accel: " << accel << " brake: " << brake << std::endl;
    float max_angular = fabs(out_twist.linear.x) * tan(max_wheel_angle * M_PI / 180) / 3.0;
    float target_angular = out_twist.angular.z;
    float target_vel = fabs(out_twist.linear.x) + accel - brake;

    // std::cout << "target_angular" << target_angular << std::endl;
    out_twist.linear.x = ((target_vel < max_vel) ? target_vel : max_vel) * back;
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
