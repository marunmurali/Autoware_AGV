#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/precision.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std;
  //Initialize the functions
  void callbackFromGnssOut(const geometry_msgs::PoseStamped &received_pose);
  void callbackFromGnssIn(const geometry_msgs::PoseStamped &received_pose);
  void callbackFromNdtpose(const geometry_msgs::PoseStamped::ConstPtr &received_pose);
  void callbackFromNdtstat(const autoware_msgs::NDTStat::ConstPtr &received_pose2);
  void callbackFromGnssPrecision(const autoware_msgs::precision::ConstPtr &received_pose3);
  void callbackFromOdom(const nav_msgs::Odometry::ConstPtr &received_Odom);
  void callbackFromImu(const sensor_msgs::Imu::ConstPtr &received_Imu);


  //Initialize messages
  geometry_msgs::PoseStamped pose_in,pose_out,pose_ndt,pose_ndt_mod,poseout;
  nav_msgs::Odometry odom_in,odom_last;
  sensor_msgs::Imu Imu_raw;

  // handle
  //ros::NodeHandle nh_;

  // publishers
  ros::Publisher pub1_;
  // subscribers
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;
  ros::Subscriber sub5_;
  ros::Subscriber sub6_;
  ros::Subscriber sub7_;
  // Initialize variables
  float ndt_score,gnss_precision;
  ros::Time score_time,precision_time,ndt_time,last_ndt_time(0);
  tf2::Quaternion quat_in,quat_last,quat_ndt,quat_delta,odom_quat;

void callbackFromGnssOut(const geometry_msgs::PoseStamped::ConstPtr &received_pose)
{
 pose_out=*received_pose;
// ROS_INFO("outdoor input x is %f\n", received_pose->pose.position.x);
 //ROS_INFO("outdoor output x is %f\n", pose_out.pose.position.x);
}
void callbackFromGnssIn(const geometry_msgs::PoseStamped::ConstPtr &received_pose)
{
 pose_in=*received_pose;
 //ROS_INFO("%s\n", pose_in.header.frame_id(0));
 //ROS_INFO("%s\n", pose_in.header.frame_id);
}
void callbackFromNdtpose(const geometry_msgs::PoseStamped::ConstPtr &received_pose)
{
pose_ndt=*received_pose;
ndt_time = received_pose->header.stamp;
}
void callbackFromNdtstat(const autoware_msgs::NDTStat::ConstPtr &received_pose2)
{
ndt_score = received_pose2->score;
score_time = received_pose2->header.stamp;
}
void callbackFromGnssPrecision(const autoware_msgs::precision::ConstPtr &received_pose3)
{
gnss_precision = received_pose3->data;
precision_time=received_pose3->header.stamp;//please make sure this line does not work when there is no new message, if not, we need to send the time along with the precision message.
}
void callbackFromOdom(const nav_msgs::Odometry::ConstPtr &received_Odom)
{
odom_last=odom_in;
odom_in=*received_Odom;
}
void callbackFromImu(const sensor_msgs::Imu::ConstPtr &received_Imu)
{
Imu_raw=*received_Imu;
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "combine_gps");
ros::NodeHandle nh_;
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);

  // setup subscriber
  sub1_ = nh_.subscribe("gnss_pose_outdoor", 10, callbackFromGnssOut);
  sub2_ = nh_.subscribe("dwm1001/tagpos", 10, callbackFromGnssIn);
  sub3_ = nh_.subscribe("ndt_pose", 10, callbackFromNdtpose);
  sub4_ = nh_.subscribe("ndt_stat", 10, callbackFromNdtstat);
  sub5_ = nh_.subscribe("gnss_precision", 10, callbackFromGnssPrecision);
  sub6_ = nh_.subscribe("odom", 10, callbackFromGnssPrecision);
  sub7_ = nh_.subscribe("imu_raw", 10, callbackFromGnssPrecision);
  //choose which position to use and put into publisher
  float ndt_score_limit = 500; // definition of input points
  nh_.getParam("ndt_score_limit", ndt_score_limit); // Get parameter from ros system
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
  //  ROS_INFO("before if \n in time  = %f \n gbs time = %f \n now time = %f \n", pose_in.header.stamp.toSec(), precision_time.toSec(), ros::Time::now().toSec());
    //ROS_INFO("in time delta  = %f \n out time delta= %f \n", (ros::Time::now().toSec()-pose_in.header.stamp.toSec()), (ros::Time::now().toSec()-precision_time.toSec()));

  if((ros::Time::now().toSec()-pose_in.header.stamp.toSec())<2)
  {
      poseout=pose_in;
      ROS_INFO("gnss received from indoor GPS.");
      //ROS_INFO("combined in input x is %f\n", pose_in.pose.position.x);
      //ROS_INFO("combined in output x is %f\n", poseout.pose.position.x);
      pub1_.publish(poseout);
  }
  else if(gnss_precision<5&&(ros::Time::now().toSec()-precision_time.toSec())<2)
  {
      poseout=pose_out;
      ROS_INFO("gnss received from outdoor GPS.");
      //ROS_INFO("combined out input x is %f\n", pose_out.pose.position.x);
      //ROS_INFO("combined out output x is %f\n", poseout.pose.position.x);
      pub1_.publish(poseout);
  }
  //else if(ndt_score > ndt_score_limit && (ros::Time::now().toSec()-score_time.toSec())<2)
  else
  {
  //extend pos ndt based on odom
  if((ndt_time.toSec()-last_ndt_time.toSec())>2&&ndt_score > ndt_score_limit)
  {
  pose_ndt_mod.pose.position.x=pose_ndt.pose.position.x;
  pose_ndt_mod.pose.position.y=pose_ndt.pose.position.x;
  pose_ndt_mod.pose.orientation=pose_ndt.pose.orientation;
  last_ndt_time=ndt_time;
  }
  pose_ndt_mod.pose.position.x = pose_ndt.pose.position.x+(odom_last.pose.pose.position.x-odom_in.pose.pose.position.x);
  pose_ndt_mod.pose.position.y = pose_ndt.pose.position.y+(odom_last.pose.pose.position.y-odom_in.pose.pose.position.y);
  tf2::convert(odom_in.pose.pose.orientation,quat_in);
  tf2::convert(odom_last.pose.pose.orientation,quat_last);
  tf2::convert(pose_ndt_mod.pose.orientation,quat_ndt);
  quat_last[3]=-quat_last[3];
  quat_delta = quat_in*quat_last;
  //quat_delta = quat_in*quat_last_inverse;
  odom_quat = quat_delta*quat_ndt;
  tf2::convert(odom_quat,pose_ndt_mod.pose.orientation);
  poseout=pose_ndt_mod;
  ROS_INFO("gnss received from ndt.");
  pub1_.publish(poseout);
  }
/*  else
  {
  ROS_INFO("no good localization available");
  }*/
  ros::spinOnce();
  loop_rate.sleep();
}
  return 0;
}
