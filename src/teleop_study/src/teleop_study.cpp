#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sound_play/sound_play.h>
#include <cmath>

class yp_teleop_study{
    private :
        ros::Subscriber sub;
        ros::Publisher pub;
        geometry_msgs::Twist twist;
		sound_play::SoundClient sc;
		bool roscore_flag ;
		bool joymode;

	public :
        yp_teleop_study();
        void joyop_loop();
		bool getmode();

    private :
        void joyCallback(const sensor_msgs::Joy &joy_msg);

};

class YpAutoware{
	private :
		ros::Subscriber sub;
		ros::Publisher pub;
		geometry_msgs::Twist twist;

	public :
		YpAutoware();
		yp_teleop_study joyop;

	private :
		void YpCallback(const geometry_msgs::TwistStamped &twistamp);
};


YpAutoware::YpAutoware(){
	ros::NodeHandle n;
    sub = n.subscribe("/twist_cmd", 1, &YpAutoware::YpCallback, this);
    pub = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);

}

void YpAutoware::YpCallback(const geometry_msgs::TwistStamped &twistamp){
	twist.linear.x = twistamp.twist.linear.x;
	twist.angular.z = twistamp.twist.angular.z;

	if(!joyop.getmode()){
		pub.publish(twist);
		ROS_INFO("automode\n");
	}
}




yp_teleop_study::yp_teleop_study(): joymode(true), roscore_flag(0){
	ros::NodeHandle n;

    sub = n.subscribe("joy", 1, &yp_teleop_study::joyCallback, this);
    pub = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);

}

void yp_teleop_study::joyop_loop() {
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        if (joymode) pub.publish(twist);
        loop_rate.sleep();
    }

return;
}


void yp_teleop_study::joyCallback(const sensor_msgs::Joy &joy_msg) {
	int dash = 1;

	if (joy_msg.buttons[0]){
		if(!joymode){
			joymode = true;
			sc.playWave("/usr/share/sounds/robot_sounds/pipe.wav");
			ROS_INFO("joymode\n");
		}else if(joymode){
			joymode = false;
			sc.playWave("/usr/share/sounds/robot_sounds/powerup.wav");
			ROS_INFO("automode\n");
		}
	}

	if (joy_msg.buttons[2]){

		if(!roscore_flag){
			std::cout << "bag_record_on" << std::endl;
			sc.playWave("/usr/share/sounds/robot_sounds/new_world.wav");
			system("bash ~/ros/catkin_ws/src/teleop/src/bag_recorder.sh &");

			roscore_flag = true;
		}else{
			std::cout << "bag_record_off" << std::endl;
			sc.playWave("/usr/share/sounds/robot_sounds/break_brick_block.wav");
			system("bash ~/ros/catkin_ws/src/teleop/src/bag_stopper.sh");
			roscore_flag = false;
		}
	}

	if (joy_msg.buttons[4] || joy_msg.buttons[5]){
		sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
		dash = 2;
	}

	if (joy_msg.buttons[4] && joy_msg.buttons[5]){
		sc.playWave("/usr/share/sounds/robot_sounds/airship_moves.wav");
		dash = 3;
	}

	if (joy_msg.buttons[1]){
		 sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
	}
	if (joy_msg.buttons[3]){
		 sc.playWave("/usr/share/sounds/robot_sounds/coin.wav");
	}

	twist.linear.x = 	dash*0.5*joy_msg.axes[3];
    twist.angular.z = 0.5*joy_msg.axes[0]*joy_msg.axes[3];

    return;

}

bool yp_teleop_study::getmode(){
	return joymode;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "yp_teleop_study");
	ros::NodeHandle n;
	YpAutoware ypauto;
	if(ypauto.joyop.getmode()){
		ypauto.joyop.joyop_loop();
	}
    return (0);

}
