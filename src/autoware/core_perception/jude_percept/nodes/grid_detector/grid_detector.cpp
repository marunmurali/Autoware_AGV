#include <algorithm> // For std::min()
#include <chrono>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <limits>
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "cluster.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>


// setup publishers
ros::Publisher grid_detector_pub; //set ros publisher to filtered_points_pub

// Initialize configuration parameters
float X_MIN = -50.0;
float X_MAX =  50.0;
float Y_MIN = -4.5;
float Y_MAX = 4.5;


void detector_cb(const autoware_msgs::CloudClusterArray &in_clusters)
{
  autoware_msgs::CloudClusterArray in_clusters_pre;
  in_clusters_pre = in_clusters;
  autoware_msgs::CloudClusterArray in_clusters_2;
  int j = 0;
  for (size_t i = 0; i < in_clusters.clusters.size(); i++)
  {
    if ((in_clusters_pre.clusters[i].bounding_box.pose.position.x >= X_MIN) && (in_clusters_pre.clusters[i].bounding_box.pose.position.x < X_MAX) && (in_clusters_pre.clusters[i].bounding_box.pose.position.y >= Y_MIN) && (in_clusters_pre.clusters[i].bounding_box.pose.position.y < Y_MAX)){

      in_clusters_2.clusters[j] = in_clusters_pre.clusters[i];
      j++;
    }
  }
  if (in_clusters_2.clusters.size() > 1) {
    int smallest_index = 0;
    double smallest_distance = sqrt(pow(in_clusters_2.clusters[0].bounding_box.pose.position.x,2.0)+pow(in_clusters_2.clusters[0].bounding_box.pose.position.y,2.0)+pow(in_clusters_2.clusters[0].bounding_box.pose.position.z,2.0));
    for (size_t i = 0; i < in_clusters_2.clusters.size(); i++)
    {
      double smallest_distance_loop = sqrt(pow(in_clusters_2.clusters[i].bounding_box.pose.position.x,2.0)+pow(in_clusters_2.clusters[i].bounding_box.pose.position.y,2.0)+pow(in_clusters_2.clusters[i].bounding_box.pose.position.z,2.0));
      if (smallest_distance_loop < smallest_distance){
        smallest_distance = smallest_distance_loop;
        smallest_index = i;
      }
    }
    autoware_msgs::DetectedObject grid_detected_objects;
    grid_detected_objects.header = in_clusters_2.header;
    grid_detected_objects.label = "unknown";
    grid_detected_objects.score = 1.;
    grid_detected_objects.space_frame = in_clusters_2.header.frame_id;
    grid_detected_objects.pose = in_clusters_2.clusters[smallest_index].bounding_box.pose;
    grid_detected_objects.dimensions = in_clusters_2.clusters[smallest_index].dimensions;
    grid_detected_objects.pointcloud = in_clusters_2.clusters[smallest_index].cloud;
    grid_detected_objects.convex_hull = in_clusters_2.clusters[smallest_index].convex_hull;
    grid_detected_objects.valid = true;

    grid_detector_pub.publish(grid_detected_objects);
  } else {
    autoware_msgs::DetectedObject grid_detected_objects;
    grid_detected_objects.header = in_clusters_2.header;
    grid_detected_objects.label = "unknown";
    grid_detected_objects.score = 1.;
    grid_detected_objects.space_frame = in_clusters_2.header.frame_id;
    grid_detected_objects.pose = in_clusters_2.clusters[0].bounding_box.pose;
    grid_detected_objects.dimensions = in_clusters_2.clusters[0].dimensions;
    grid_detected_objects.pointcloud = in_clusters_2.clusters[0].cloud;
    grid_detected_objects.convex_hull = in_clusters_2.clusters[0].convex_hull;
    grid_detected_objects.valid = true;

    grid_detector_pub.publish(grid_detected_objects);
  }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_detector"); // initialize node

  ros::NodeHandle nh; // definition of node handler
  ros::NodeHandle private_nh("~"); // node handler

  std::string INPUT_TOPIC = "/detection/lidar_detector/cloud_clusters"; // definition of in_detection
  private_nh.getParam("input_topic", INPUT_TOPIC); // Get parameter from ros system

  std::string OUTPUT_TOPIC = "/detection/lidar_detector/objects/grid"; // definition of output topic
  private_nh.getParam("output_topic", OUTPUT_TOPIC); // Get parameter from ros system

  private_nh.getParam("x_min", X_MIN);
  private_nh.getParam("x_max", X_MAX);
  private_nh.getParam("y_min", Y_MIN);
  private_nh.getParam("y_max", Y_MAX);

  // Publishers
  grid_detector_pub = nh.advertise<autoware_msgs::DetectedObjectArray>(OUTPUT_TOPIC, 10); //setup publisher

  // Subscribers
  ros::Subscriber scan_sub = nh.subscribe(INPUT_TOPIC, 10, detector_cb); // setup subsriber

  ros::spin(); //loop

  return 0;
}
