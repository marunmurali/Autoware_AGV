#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>//Arun
#include "std_msgs/Int32.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


#include <cmath>

#include "autoware_config_msgs/ConfigRealTimeObstacleSize.h"

geometry_msgs::Twist tw;
std_msgs::Int32 tw2;//Arun
ros::Publisher  pub1;
ros::Publisher  pub2;//Arun
int obstacle_size;
//int number_of_points;
size_t number_of_points;

static void config_callback(const autoware_config_msgs::ConfigRealTimeObstacleSize::ConstPtr& input)
{
  obstacle_size = input->obstacle_size;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& obstacle_points_filtered)
{

// pcl Container for filtered data 
 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

// Convert to PCL data type
 pcl::fromROSMsg(*obstacle_points_filtered, *cloud);
 //number_of_points = pcl::size(cloud);
 number_of_points = cloud->size();
 tw2.data=number_of_points;//Arun
 pub2.publish(tw2);//Arun
 if (number_of_points > obstacle_size) 
 {
   tw.linear.x = 0;
   tw.angular.z = 0;
   pub1.publish(tw);
 }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_stop"); // initialize node

  ros::NodeHandle nh; // definition of node handler
  ros::NodeHandle private_nh("~"); // node handler

  std::string POINTS_TOPIC = "/obstacle_points_filtered"; // definition of input points
  private_nh.getParam("points_topic", POINTS_TOPIC); // Get parameter from ros system


  // Publishers
  pub1 = nh.advertise<geometry_msgs::Twist>("obstacle_twist", 10); //setup publisher
  pub2 = nh.advertise<std_msgs::Int32>("points_on_obstacle", 10);

  // Subscribers
  ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 10, cloud_cb); // setup subsriber
  ros::Subscriber config_sub = nh.subscribe("config/obstacle_stop", 10, config_callback);

  ros::spin(); //loop

  return 0;
}
