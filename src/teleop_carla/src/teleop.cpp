#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

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
    ros::Publisher pub_vehicle_control;

    float pub_rate;


    geometry_msgs::Twist m_autoware_twist;
    // geometry_msgs::Twist joy_twist;
    geometry_msgs::Twist m_current_twist;
    carla_msgs::CarlaEgoVehicleControl m_vehicle_cmd;
    float m_throttle;
    float m_brake;
    float m_manual_omega;
    // float max_radius;
    ros::Timer timer;

    // dynamic_reconfigure params
    float m_max_vel;
    float m_max_wheel_angle;
    float m_max_accel;
    float m_max_decel;
    float m_natural_decel;
    int m_controller;
    int m_control_type;
    bool m_autonomous_mode;
    bool m_rosbag_flag;
    bool m_back;

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
    float calcVelChange(float current_vel, float autonomous_vel, float throttle, float brake, float natural_decel, float max_vel, float max_throttle, float max_decel);
    float calcOmega(float current_vel, float autonomous_omega, float manual_omega, float max_wheel_angle );
};


Teleop::Teleop(): m_throttle(0.0), m_brake(0.0), m_back(false), pub_rate(0.1)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&Teleop::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_joy = n.subscribe("/joy", 1, &Teleop::joyCallback, this);
    sub_twist = n.subscribe("/twist_cmd", 1, &Teleop::twistCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 1, &Teleop::odomCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist_cmd", 1);
    pub_vehicle_control = n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd_manual", 1);
    ros::Duration(1).sleep();
    timer = n.createTimer(ros::Duration(pub_rate), &Teleop::timerCallback, this);
}


void Teleop::callbackDynamicReconfigure(teleop_carla::teleopConfig &config, uint32_t lebel)
{
    m_max_vel = config.max_speed;
    m_max_wheel_angle = 90.0 - config.max_steer;
    m_max_accel = config.acceleration_coefficient * 9.8 * pub_rate * 3.6;
    m_max_decel = config.deceleration_coefficient * 9.8 * pub_rate  * 3.6;
    m_natural_decel = config.natural_deceleration_coefficient * 9.8 * pub_rate  * 3.6;
    m_autonomous_mode = config.autonomous_mode?true:false;
    m_rosbag_flag = config.rosbag_flag?true:false;
    m_controller = config.controller;
    m_control_type = config.control_type;
}

void Teleop::joyCallback(const sensor_msgs::Joy &in_joy)
{
    if (m_controller == 0)
    {
        m_throttle = (1.0 - in_joy.axes[5]) * 0.5;
        m_brake = (1.0 - in_joy.axes[2]) * 0.5;
        m_manual_omega = -m_current_twist.linear.x * tan((90.0 - m_max_wheel_angle) * in_joy.axes[0] * M_PI / 180) / 3.0;

        if(in_joy.buttons[1] && fabs(m_current_twist.linear.x) < 2.0)
        {
            m_back = !m_back;
            m_autonomous_mode = false;
        }

        if(in_joy.buttons[2])
        {
            m_autonomous_mode = !m_autonomous_mode;
        }

        // START [7]
        if (in_joy.buttons[7])
        {
            m_rosbag_flag = !m_rosbag_flag;

            if(!m_rosbag_flag)
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
    else if(m_controller == 1)
    {
        m_throttle = (1.0 + in_joy.axes[2]) * 0.5;
        m_brake = (1.0 + in_joy.axes[3]) * 0.5;
        m_manual_omega = -m_current_twist.linear.x * tan((90.0 - m_max_wheel_angle) * in_joy.axes[0] * M_PI / 180) / 3.0;

        if(in_joy.buttons[3])
            m_autonomous_mode = !m_autonomous_mode;

        if(in_joy.buttons[2] && !m_autonomous_mode)
        {
            m_back = !m_back;
            m_autonomous_mode = false;
        }
        // START [7]
        if (in_joy.buttons[1])
        {
            m_rosbag_flag = !m_rosbag_flag;
            if(!m_rosbag_flag)
            {
                ROS_INFO("bag_record_on");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_recorder.sh &");
            }else
            {
                ROS_INFO("bag_record_off");
                // system("bash ~/Program/Ros/master_study_ws/src/teleop_study/src/bag_stopper.sh &");
            }
        }

        m_vehicle_cmd.header.stamp = ros::Time::now();
        m_vehicle_cmd.throttle = m_throttle;
        m_vehicle_cmd.steer = in_joy.axes[0];
        m_vehicle_cmd.brake = m_brake;
        m_vehicle_cmd.reverse = m_back;
        m_vehicle_cmd.gear = int(m_vehicle_cmd.manual_gear_shift + in_joy.buttons[4] - in_joy.buttons[5]) % 4 + 1;
    }
}


void Teleop::twistCallback(const geometry_msgs::TwistStamped &in_twist)
{
    if (m_autonomous_mode)
    {
        m_autoware_twist = in_twist.twist;
        m_autoware_twist.angular.z = -in_twist.twist.angular.z;
    }
}


void Teleop::odomCallback(const nav_msgs::Odometry &in_odom)
{
    m_current_twist = in_odom.twist.twist;
}


void Teleop::timerCallback(const ros::TimerEvent&)
{
    if (m_control_type == 0)
    {
        geometry_msgs::Twist out_twist;
        float vel_change = calcVelChange(m_current_twist.linear.x, m_autoware_twist.linear.x, m_throttle, m_brake, m_natural_decel, m_max_vel,m_max_accel, m_max_decel);
        if (m_back)
        vel_change *= -1;
        out_twist.linear.x = m_current_twist.linear.x + vel_change;
        if ((m_back && out_twist.linear.x > 0.0) || (!m_back && out_twist.linear.x < 0.0))
        {
            std::cout << "sign changed unexpectedly" << m_back << m_control_type << out_twist.linear.x << std::endl;
            out_twist.linear.x = 0.0;
        }
        out_twist.angular.z = calcOmega(m_current_twist.linear.x, m_autoware_twist.angular.z, m_manual_omega, m_max_wheel_angle);
        pub_twist.publish(out_twist);
        // m_autoware_twist.linear.x = 0.0;
    }
    else if (m_control_type == 1)
    {
        pub_vehicle_control.publish(m_vehicle_cmd);
        pub_twist.publish(m_autoware_twist);
    }
}


float Teleop::calcVelChange(const float current_vel, const float autonomous_vel, const float throttle, const float brake, const float natural_decel, const float max_vel, const float max_accel, const float max_decel)
{
    float vel_change;

    // when no manual override
    if (brake == 0.0 && throttle == 0.0)
    {
        if (autonomous_vel == 0.0)
            return -natural_decel;

        else
        {
            vel_change = autonomous_vel - current_vel;
            // cut velocity change with threshold
            if (vel_change < -max_decel || max_accel < vel_change)
            {
                std::cout << "velchange over max" << std::endl;
                return (vel_change > 0.0) ? max_accel : -max_decel;
            }
        }
    }
    // when manual override autonomous operation is ignored
    else
        vel_change = throttle * max_accel - brake * max_decel;

    if (fabs(current_vel) > max_vel)
    {
        std::cout << "vel is over max" << std::endl;
        return 0.0;
    }
    return vel_change;
}


float Teleop::calcOmega(const float current_vel, const float autonomous_omega, const float manual_omega, const float max_wheel_angle )
{
    float max_angular = fabs(current_vel) * tan(max_wheel_angle * M_PI / 180) / 3.0;
    if (manual_omega == 0.0)
        return (fabs(autonomous_omega) < max_angular) ? -autonomous_omega : (autonomous_omega < 0.0) ? max_angular : -max_angular;
    else
        return (fabs(manual_omega) < max_angular) ? manual_omega : (manual_omega < 0.0) ? -max_angular : max_angular;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    Teleop teleop;
    ros::spin();
    return(0);
}
