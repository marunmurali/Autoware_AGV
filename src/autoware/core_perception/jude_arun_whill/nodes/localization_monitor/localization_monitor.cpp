#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/precision.h>
using namespace std;



// setup publishers
ros::Publisher  pub1;

// setup Subscriber
ros::Subscriber sub1;

// Initialize variables
geometry_msgs::Twist tw;
float ndt_score;
float ndt_score_to_stop = 0.8;
double stop_time = 0.5;
double last_ndt_time = 0.0;
int ndtIsBad;
int callbackCalled;

//Initialize the functions
void callbackFromNdtstat(const autoware_msgs::NDTStat::ConstPtr &ndtstatus);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor");

  callbackCalled = 0;

  // Node handlers
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Get parameters from ros
  private_nh.getParam("ndt_score_to_stop", ndt_score_to_stop);
  private_nh.getParam("stop_time", stop_time);

  // publishers and subscribers
  pub1 = nh.advertise<geometry_msgs::Twist>("obstacle_twist", 10);
  sub1 = nh.subscribe("ndt_stat", 10, callbackFromNdtstat);

  // Looping non-callback function calculations

  ros::Rate loop_rate(10);
  while(ros::ok())
  {

    if (callbackCalled == 0)
    {
      tw.linear.x =  0;
      tw.angular.z = 0;

      pub1.publish(tw);
      ROS_INFO("NDT NOT AVAILABLE, SO ROBOT STOPPED");
    }

    if (ndtIsBad == 1)
    {
      tw.linear.x =  0;
      tw.angular.z = 0;

      pub1.publish(tw);
      ROS_INFO("NDT STATUS: BAD, ROBOT STOPPED");
    }
    if (ndtIsBad == 2)
    {

      ROS_INFO("NDT STATUS: OK");
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}


void callbackFromNdtstat(const autoware_msgs::NDTStat::ConstPtr &ndt_status)
{
  ndt_score = ndt_status -> score;
  if ((ros::Time::now().toSec() - last_ndt_time) < 0){
    last_ndt_time = ros::Time::now().toSec() - stop_time - 0.1;
  }
  if ((last_ndt_time==0) && (ndt_score > ndt_score_to_stop)){

    last_ndt_time = ros::Time::now().toSec()-stop_time-0.1;

  }
  if(((ros::Time::now().toSec() - last_ndt_time ) > stop_time) && (ndt_score > ndt_score_to_stop)){

    ndtIsBad = 1;

    last_ndt_time = ros::Time::now().toSec()-stop_time-0.1;

  }
  else
  {
    ndtIsBad = 2;
   }
  callbackCalled = 1;
}
