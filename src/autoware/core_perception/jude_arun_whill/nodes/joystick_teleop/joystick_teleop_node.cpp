#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

const static int DEFAULT_AXIS_LINEAR_ID = 1;
const static int DEFAULT_AXIS_ANGULAR_ID = 0;
const static int DEFAULT_ACTIVATE_BUTTON_ID = 0;
const static double DEFAULT_TIMEOUT_DURATION = 0.1; // seconds

class JoystickTeleopNode
{
public:
    JoystickTeleopNode();
    void run();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;

    int axis_linear_id, axis_angular_id, activate_button_id, axis_button_vertical_id;
    double max_linear_vel, max_angular_vel;
    bool activate_button_is_pressed;
    bool warning_printed;
    ros::Time last_time;
    double timeout_duration;
    geometry_msgs::Twist twist;
};


JoystickTeleopNode::JoystickTeleopNode():
    axis_linear_id(DEFAULT_AXIS_LINEAR_ID),
    axis_angular_id(DEFAULT_AXIS_ANGULAR_ID),
    activate_button_id(DEFAULT_ACTIVATE_BUTTON_ID),
    max_linear_vel(0),
    max_angular_vel(0),
    activate_button_is_pressed(false),
    warning_printed(true)
{
    ros::NodeHandle pnh("~");

    //update parameters from Parameter Server
    pnh.param("activate_button_id", activate_button_id, activate_button_id);
    pnh.param("axis_linear_id", axis_linear_id, axis_linear_id);
    pnh.param("axis_angular_id", axis_angular_id, axis_angular_id);
    pnh.param("axis_button_vertical_id", axis_button_vertical_id, axis_button_vertical_id);
    pnh.param("max_linear_vel", max_linear_vel, max_linear_vel);
    pnh.param("max_angular_vel", max_angular_vel, max_angular_vel);
    pnh.param("timeout_duration", timeout_duration, DEFAULT_TIMEOUT_DURATION);

    vel_pub = pnh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    joy_sub = pnh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickTeleopNode::joyCallback, this); //will subscribe to either private or global

    last_time = ros::Time::now();
}

void JoystickTeleopNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //ROS_INFO("joy %d %f %f", joy->buttons[activate_button_id_], joy->axes[axis_linear_id], joy->axes[ax_ang_id_]);

    warning_printed = false;
    //if(!warning_printed)
    //    warning_printed = true;
    //geometry_msgs::Twist twist;
    if(joy->buttons[activate_button_id] == 1)
    {
        activate_button_is_pressed = true;
        //twist.linear.x = max_linear_vel * joy->axes[axis_linear_id];
        //twist.angular.z = max_angular_vel * joy->axes[axis_angular_id];
        //vel_pub.publish(twist);
    }
    else
    {
        if(activate_button_is_pressed)
        {
            twist.linear.x = 0;
            twist.angular.z = 0;
            vel_pub.publish(twist);
            activate_button_is_pressed = false;
        }
    }

    if(activate_button_is_pressed)
    {
        twist.linear.x = max_linear_vel * joy->axes[axis_linear_id];
        twist.angular.z = max_angular_vel * joy->axes[axis_angular_id];

        if(joy->axes[axis_button_vertical_id] == 1){
          twist.linear.x = max_linear_vel;
          twist.angular.z = 0;
        }
        if(joy->axes[axis_button_vertical_id] == -1){
          twist.linear.x = -max_linear_vel;
          twist.angular.z = 0;
        }
    }
    last_time = ros::Time::now();
}

void JoystickTeleopNode::run()
{
    //if(timeout_duration <= 0)
    //{
    //	ROS_WARN("timeout disabled!");
    //	ros::spin();
    //}
    //else
    //{
    	ros::Duration d = ros::Duration(0.005);
    	double delta_time = 0;
    	while (ros::ok())
    	{
            if(timeout_duration > 0)
            {
            	delta_time = ros::Time::now().toSec() - last_time.toSec();
            	if(delta_time > timeout_duration)
            	{
                		//geometry_msgs::Twist twist;
                		twist.linear.x = 0;
                		twist.angular.z = 0;
                		if(!warning_printed)
                        {
                    		ROS_ERROR("THE JOYSTICK DID NOT REPORT ANY DATA IN %lf seconds", delta_time);
                            warning_printed = true;
                        }
            	}
            }

            if(activate_button_is_pressed)
                vel_pub.publish(twist);
        	ros::spinOnce();
        	d.sleep();
    	}
    //}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "JoystickTeleopNode");
    JoystickTeleopNode node;
    node.run();
    return 0;
}
