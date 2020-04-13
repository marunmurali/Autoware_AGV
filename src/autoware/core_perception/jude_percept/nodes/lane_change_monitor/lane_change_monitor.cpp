#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>

#include <ros/ros.h>

#include "autoware_config_msgs/ConfigLaneChangeMonitor.h"

ros::Publisher lane_change_monitor_pub; //set ros publisher to filtered_points_pub


// Initialize realtime configuration parameters
int lane_change_monitor_state = 0;


static void config_callback(const autoware_config_msgs::ConfigLaneChangeMonitor::ConstPtr& input)
{
  lane_change_monitor_state = input->lane_change_monitor_state;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_change_monitor"); // initialize node

  ros::NodeHandle nh; // definition of node handler
  ros::NodeHandle private_nh("~"); // node handler


  // Subscribers
  ros::Subscriber config_sub = nh.subscribe("config/lane_change_monitor", 10, config_callback);

  ros::spin(); //loop

  return 0;
}

