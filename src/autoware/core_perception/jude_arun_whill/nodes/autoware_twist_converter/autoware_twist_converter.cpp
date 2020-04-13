#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>


geometry_msgs::Twist tw;
ros::Publisher  pub1;

void auto_twist(const geometry_msgs::TwistStamped::ConstPtr &received_twist) 
{

  tw.linear.x =  received_twist->twist.linear.x;
  tw.angular.z = received_twist->twist.angular.z;

  pub1.publish(tw);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "autoware_twist_converter");

  ros::NodeHandle nh;

  pub1 = nh.advertise<geometry_msgs::Twist>("autoware_twist", 10);
  ros::Subscriber sub1 = nh.subscribe("twist_raw", 10, auto_twist);
  

  ros::spin();
   

return 0;
}

