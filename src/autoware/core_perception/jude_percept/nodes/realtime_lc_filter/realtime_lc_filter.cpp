#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "autoware_config_msgs/ConfigRealTimeLaneChangeProcessor.h"

ros::Publisher filtered_points_pub; //set ros publisher to filtered_points_pub

// Initialize realtime configuration parameters
float x_min = -50.0;
float x_max = 50.0;
float y_min = -4.5;
float y_max = 4.5;
float z_min = -4.5;
float z_max = 4.5;

static void config_callback(const autoware_config_msgs::ConfigRealTimeLaneChangeProcessor::ConstPtr& input)
{
  x_min = input->x_min;
  x_max = input->x_max;
  y_min = input->y_min;
  y_max = input->y_max;
  z_min = input->z_min;
  z_max = input->z_max;
}


void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& points_raw);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "realtime_lc_filter"); // initialize node

  ros::NodeHandle nh; // definition of node handler
  ros::NodeHandle private_nh("~"); // node handler

  std::string POINTS_TOPIC = "/points_raw"; // definition of input points
  private_nh.getParam("points_topic", POINTS_TOPIC); // Get parameter from ros system

  std::string OUTPUT_TOPIC = "/obstacle_points_filtered"; // definition of output topic
  private_nh.getParam("output_topic", OUTPUT_TOPIC); // Get parameter from ros system

 /*
  private_nh.getParam("x_min", X_MIN);
  private_nh.getParam("x_max", X_MAX);
  private_nh.getParam("y_min", Y_MIN);
  private_nh.getParam("y_max", Y_MAX);
*/

  // Publishers
  filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>(OUTPUT_TOPIC, 10); //setup publisher

  // Subscribers
  ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 10, cloud_cb); // setup subsriber
  ros::Subscriber config_sub = nh.subscribe("config/realtime_lc_filter", 10, config_callback);

  ros::spin(); //loop

  return 0;
}

void
 cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& points_raw)
{
  // Container for original & filtered data 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_pre(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_pre2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // Convert to PCL data type
  pcl::fromROSMsg(*points_raw, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_min, x_max);
  pass.filter (*cloud_filtered_pre);
  pass.setInputCloud (cloud_filtered_pre);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_min, y_max);
  pass.filter (*cloud_filtered_pre2);
  pass.setInputCloud (cloud_filtered_pre2);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min, z_max);
  pass.filter (*cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 LC_points;
  pcl::toROSMsg(*cloud_filtered, LC_points);
  LC_points.header = points_raw->header; //gives subscribed topic header to output 


 //Publish the data 
 filtered_points_pub.publish (LC_points);
}

