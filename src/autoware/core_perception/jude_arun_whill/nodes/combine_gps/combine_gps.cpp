#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/precision.h>
#include <iostream>
#include <time.h>
using namespace std;
  //Initialize the functions
  void callbackFromGnssOut(const geometry_msgs::PoseStamped &received_pose);
  void callbackFromGnssIn(const geometry_msgs::PoseStamped &received_pose);
  void callbackFromNdtpose(const geometry_msgs::PoseStamped::ConstPtr &received_pose);
  void callbackFromNdtstat(const autoware_msgs::NDTStat::ConstPtr &received_pose2);
  void callbackFromGnssPrecision(const autoware_msgs::precision::ConstPtr &received_pose3);

  //Initialize messages
  geometry_msgs::PoseStamped pose_in,pose_out,pose_ndt,poseout;

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
  // Initialize variables
  float ndt_score,gnss_precision;
  ros::Time score_time,precision_time;

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
  //choose which position to use and put into publisher
  float ndt_score_limit = 500; // definition of input points
  nh_.getParam("ndt_score_limit", ndt_score_limit); // Get parameter from ros system
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
  //  ROS_INFO("before if \n in time  = %f \n gbs time = %f \n now time = %f \n", pose_in.header.stamp.toSec(), precision_time.toSec(), ros::Time::now().toSec());
    //ROS_INFO("in time delta  = %f \n out time delta= %f \n", (ros::Time::now().toSec()-pose_in.header.stamp.toSec()), (ros::Time::now().toSec()-precision_time.toSec()));

  if((ros::Time::now().toSec()-pose_in.header.stamp.toSec())<5)
  {
      poseout=pose_in;
      ROS_INFO("gnss received from indoor GPS.");
      //ROS_INFO("combined in input x is %f\n", pose_in.pose.position.x);
      //ROS_INFO("combined in output x is %f\n", poseout.pose.position.x);
      pub1_.publish(poseout);
  }
  else if(gnss_precision<5&&(ros::Time::now().toSec()-precision_time.toSec())<10)
  {
      poseout=pose_out;
      ROS_INFO("gnss received from outdoor GPS.");
      //ROS_INFO("combined out input x is %f\n", pose_out.pose.position.x);
      //ROS_INFO("combined out output x is %f\n", poseout.pose.position.x);
      pub1_.publish(poseout);
  }
  else if(ndt_score > ndt_score_limit && (ros::Time::now().toSec()-score_time.toSec())<3)
  {
  poseout=pose_ndt;
  ROS_INFO("gnss received from ndt.");
  pub1_.publish(poseout);
  }
  else
  {
  ROS_INFO("no good localization available");
  }
  ros::spinOnce();
  loop_rate.sleep();
}
  return 0;
}
