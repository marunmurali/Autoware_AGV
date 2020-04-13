#include <ros/ros.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <autoware_can_msgs/CANData.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

static autoware_can_msgs::CANData candata;

static std::string windowName = "CAN Visualizer";
static cv::Mat steerImg = cv::imread("/home/autoware2/autoware.ai/src/drivers/awf_drivers/kvaser/nodes/can_visualizer/steeringWheel.png",-1);

cv::Mat rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point2f pt(src.cols/2., src.rows/2.);    
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
    return dst;
}

void initializeWindow(cv::Mat background){
    cv::namedWindow(windowName);    
    
    cv::Mat background_roi = background(cv::Rect(0,0,400,400));
    steerImg.copyTo(background_roi);
    
    cv::rectangle(background, cv::Point(420,190), cv::Point(480, 390), cv::Scalar(0,0,200), 3, CV_AA);
    cv::rectangle(background, cv::Point(520,190), cv::Point(580, 390), cv::Scalar(0,200,0), 3, CV_AA);
    
    cv::putText(background, "kph", cv::Point(520, 150), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(100,255,200), 2, CV_AA);   
    
    cv::rectangle(background, cv::Point(490, 15), cv::Point(590,65), cv::Scalar(100,255,200), 2, CV_AA);
    cv::putText(background, "Engine", cv::Point(430, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,255,200), 2, CV_AA);
}

void canVisualize(const autoware_can_msgs::CANData& data){
    cv::Mat background(400, 600, CV_8UC3, cv::Scalar(0,0,0));
    
    initializeWindow(background);
    
    //steer
    int steer = data.steer;
    cv::Mat steerImgRotated = rotate(steerImg, steer);
    cv::Mat background_roi = background(cv::Rect(0,0,400,400));
    steerImgRotated.copyTo(background_roi);
        
    if(steer < 0){
    	cv::putText(background, "CW", cv::Point(120, 380), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,200,100), 2, CV_AA);
	steer=-steer;
    }
    else{
	cv::putText(background, "CCW", cv::Point(90, 380), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,200,100), 2, CV_AA);
    }
    char steer_str[5];
    sprintf(steer_str, "%d", steer);
    cv::putText(background, steer_str, cv::Point(180, 380), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,200,100), 2, CV_AA);
    cv::putText(background, "deg", cv::Point(250, 380), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,200,100), 2, CV_AA);
    
    //brake
    cv::rectangle(background, cv::Point(420, 190+(100-data.brake)*200/100), cv::Point(480, 390), cv::Scalar(0,0,200), -1, CV_AA);
    char brake_str[4];
    sprintf(brake_str, "%u", data.brake);
    cv::putText(background, brake_str, cv::Point(430, 380), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,200,100), 2, CV_AA);    
    
    //throttle
    cv::rectangle(background, cv::Point(520, 190+(100-data.throttle)*200/100), cv::Point(580, 390), cv::Scalar(0,200,0), -1, CV_AA);
    char throttle_str[4];
    sprintf(throttle_str, "%u", data.throttle);
    cv::putText(background, throttle_str, cv::Point(530, 380), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,200,100), 2, CV_AA);
    
    //speed
    char speed[3];
    sprintf(speed, "%u", data.speed3);
    
    cv::putText(background, speed, cv::Point(440, 150), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(100,255,200), 2, CV_AA);   
    
    //shift(0, 8, 15, 72, 79)
    char shift[4];
    switch(data.shift){
        case 0:
            sprintf(shift, "%s", "P  ");
            break;
        case 8: //Engine off
            sprintf(shift, "%s", "OFF");
            break;
        case 15: //Engine on
            sprintf(shift, "%s", "ON ");
            break;
        case 79:
            sprintf(shift, "%s", "R  ");
            break;
    }
    cv::putText(background, shift, cv::Point(495, 55), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(100,255,200), 2, CV_AA); 
    
    cv::imshow(windowName, background);
    cv::waitKey(1);
}

void chatterCallback(const autoware_can_msgs::CANData::ConstPtr& msg)
{
    candata = *msg;
    canVisualize(candata);
}

int main (int argc, char *argv[]){
    ros::init(argc, argv, "can_visualizer");
    ros::NodeHandle nh("~");

    cv::cvtColor(steerImg, steerImg, CV_BGRA2BGR);
    
    candata.steer = 0;
    candata.throttle = 0;
    candata.brake = 0;
    candata.speed3 = 0;
    candata.shift = 0;
    
    cv::namedWindow(windowName);
    canVisualize(candata);
    
    ros::Subscriber sub = nh.subscribe("/can_data", 10, chatterCallback);
    ros::spin();
    
    cv::destroyWindow(windowName);
}
