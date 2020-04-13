#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


ros::Publisher filtered_points_pub; //set ros publisher to filtered_points_pub

// Initialize configuration parameters
float X_MIN = -50.0;
float X_MAX =  50.0;
float Y_MIN = -4.5;
float Y_MAX = 4.5;

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& points_raw);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_filter"); // initialize node

  ros::NodeHandle nh; // definition of node handler
  ros::NodeHandle private_nh("~"); // node handler

  std::string POINTS_TOPIC = "/points_raw"; // definition of input points
  private_nh.getParam("points_topic", POINTS_TOPIC); // Get parameter from ros system

  std::string OUTPUT_TOPIC = "/filtered_points"; // definition of output topic
  private_nh.getParam("output_topic", OUTPUT_TOPIC); // Get parameter from ros system

  private_nh.getParam("x_min", X_MIN);
  private_nh.getParam("x_max", X_MAX);
  private_nh.getParam("y_min", Y_MIN);
  private_nh.getParam("y_max", Y_MAX);

  // Publishers
  filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>(OUTPUT_TOPIC, 10); //setup publisher

  // Subscribers
  ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 10, cloud_cb); // setup subsriber

  ros::spin(); //loop

  return 0;
}

void
 cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& points_raw)
{
  // Container for original & filtered data 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_pre(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // Convert to PCL data type
  pcl::fromROSMsg(*points_raw, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (X_MIN, X_MAX);
  pass.filter (*cloud_filtered_pre);
  pass.setInputCloud (cloud_filtered_pre);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (Y_MIN, Y_MAX);
  pass.filter (*cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 LC_points;
  pcl::toROSMsg(*cloud_filtered, LC_points);
  LC_points.header = points_raw->header; //gives subscribed topic header to output 


 //Publish the data 
 filtered_points_pub.publish (LC_points);
}

