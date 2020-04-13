//Created by Shunichiro Sugiyama 2018

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define PRIORITY_HIGH 2
#define PRIORITY_MID  1
#define PRIORITY_LOW  0

ros::Publisher cmd_vel_pub;
ros::Time last_recieved_time;
int last_priority = PRIORITY_LOW;
double HOLD_SEC = 1.0;

void publish_cmd_vel(geometry_msgs::Twist cmd, int priority){
	
	if( last_priority > priority &&
			ros::Time::now() - last_recieved_time < ros::Duration(HOLD_SEC) )
	{
		return;
	}

	cmd_vel_pub.publish(cmd);
	last_recieved_time = ros::Time::now();
	last_priority = priority;
}

void cmd_vel_high_callback(geometry_msgs::Twist cmd_vel_in){
	publish_cmd_vel(cmd_vel_in, PRIORITY_HIGH);
}

void cmd_vel_mid_callback(geometry_msgs::Twist cmd_vel_in){
	publish_cmd_vel(cmd_vel_in, PRIORITY_MID);
}

void cmd_vel_low_callback(geometry_msgs::Twist cmd_vel_in){
	publish_cmd_vel(cmd_vel_in, PRIORITY_LOW);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cmd_vel_multiplexer");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(50);

	nh.param("hold_sec",HOLD_SEC,1.0);

  ros::Subscriber cmd_vel_high = nh.subscribe<geometry_msgs::Twist>("cmd_vel_priority_high", 1, cmd_vel_high_callback);
	ros::Subscriber cmd_vel_mid = nh.subscribe<geometry_msgs::Twist>("cmd_vel_priority_mid", 1, cmd_vel_mid_callback);
	ros::Subscriber cmd_vel_low = nh.subscribe<geometry_msgs::Twist>("cmd_vel_priority_low", 1, cmd_vel_low_callback);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);

  ros::spin();
}
