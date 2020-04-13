#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"
#include <autoware_can_msgs/CANData.h>

static ros::Publisher pub;
static ros::Subscriber sub;

static autoware_can_msgs::CANData candata;

void chatterCallback(const autoware_can_msgs::CANPacket::ConstPtr& msg)
{
  unsigned short w;
  static int enc_sum;
  short diff;
  static short steer,shift,speed2,speed3,brake;
  static char  speed;
  static short enc,enc_p,enc_diff;
  static short wheel1,wheel2,wheel3,wheel4;
  static short gyro,accy,accz;
  static char accx; 
  static short throttle;
  static FILE* log_fp;

  int changed=0;

  candata.header = msg->header;
    
  if(msg->id==0x245){ //id:581
    w=msg->dat[2];
    throttle=w/2; //0~200 -> 0~100%
    changed=1;
    candata.throttle = throttle;
  }

  if(msg->id==0x24){
    w=msg->dat[0]*256+msg->dat[1];
    gyro=w;
    w=msg->dat[2]*256+msg->dat[3];
    accx=w;
    w=msg->dat[4]*256+msg->dat[5];
    accy=w;
    w=msg->dat[7];
    accz=w;
    changed=1;
    candata.gyro = gyro;
    candata.accx = accx;
    candata.accy = accy;
    candata.accz = accz;
  }
  if(msg->id==0x25){
    w=msg->dat[0]*256+msg->dat[1];
    steer=w;
    if(steer>2560) steer -= 4096;
    changed=1;
    candata.steer = steer*1.5;
  }

  if(msg->id==0xaa){
    w=msg->dat[0]*256+msg->dat[1];
    wheel1=w;
    w=msg->dat[2]*256+msg->dat[3];
    wheel2=w;
    w=msg->dat[4]*256+msg->dat[5];
    wheel3=w;
    w=msg->dat[6]*256+msg->dat[7];
    wheel4=w;
    changed=1;
    candata.wheel1 = wheel1;
    candata.wheel2 = wheel2;
    candata.wheel3 = wheel3;
    candata.wheel4 = wheel4;
  }
  if(msg->id==0xb4){
    w=msg->dat[5]*256+msg->dat[6];
    float tmp = (float)w/1024.*10.+0.5;
    speed3=(unsigned short)tmp; //km/h
    changed=1;
    candata.speed3 = speed3;
  }
  if(msg->id==0x224){
    w=msg->dat[4]*256+msg->dat[5];
    brake=w;
    changed=1;
    candata.brake = brake/15.5;
  }
  if(msg->id==0x127){
    shift=msg->dat[3];
    speed=msg->dat[4];
    changed=1;
    candata.shift = shift;
    candata.speed = speed;
  }
  if(msg->id==0x230){
    w=msg->dat[0]*256+msg->dat[1];
    enc_p=enc;
    enc=w;
    diff=enc-enc_p;
    enc_diff=diff;
    enc_sum+=diff;
    changed=1;
    candata.enc_sum = enc_sum;
    candata.enc_diff = enc_diff;    
  }
  if(changed){
    if(!log_fp)log_fp=fopen("/tmp/can_log","w");
    fprintf(log_fp,"%f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
	    msg->header.stamp.toSec(),msg->time,steer,shift,speed,speed2,speed3,enc_sum,enc_diff,brake,
	    wheel1,wheel2,wheel3,wheel4,
	    throttle,accx,accy,accz,gyro);
    pub.publish(candata);
  }  
}


int main (int argc, char *argv[]){
  autoware_can_msgs::CANPacket candat;

  ros::init(argc, argv, "can_converter");
  ros::NodeHandle n;

    pub = n.advertise<autoware_can_msgs::CANData>("can_data", 10);  
    sub = n.subscribe("can_raw", 1000, chatterCallback);

  ros::spin();
}
