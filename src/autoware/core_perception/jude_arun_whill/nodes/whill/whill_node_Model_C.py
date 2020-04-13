#!/usr/bin/env python

#Created on Monday May 14th 2018
#author: Luis Yoichi Morales
#import roslib; roslib.load_manifest('whill_driver')
import rospy
import time
import std_msgs.msg
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import sin, cos, pi

from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import*
from time import sleep

#geometry_msgs::Twist::ConstPtr

from WHILL_Command_Model_C import WHILL
from WHILL_Command_Model_C import to_bytes
TREAD = 0.5

dv = 0
dw = 0


def cmd_callback(cmd):
    global dv
    global dw
    dv = cmd.linear.x
    dw = cmd.angular.z
    #print "callback", dv, dw

#Transforms velocity commands to joystick commands (values extracted with a person weighting 70Kg)
#dv:velocity is in m/sec [-0.5:1.6];  dw:angular velocity rad/sec [-1:1];   joystick commands [100:-100]
#Parameters a,b,c,d were computed using nonlinear least-squares (NLLS) Marquardt-Levenberg algorithm.
def set_velocity_to_joystick_range( dv, dw ):
    v_a = 6.35
    v_b = -19.16
    v_c = 71.04
    v_d = 4.54
    jv = v_a*dv**3  + v_b*dv**2 + v_c*dv + v_d
    if jv <= 4.54 and jv >= -4.54 :
        jv = 0

    w_a = 9.39201
    w_b = -3.7967
    w_c = -96.1935
    w_d = 0.17726
    jw = w_a*dw**3  + w_b*dw**2 + w_c*dw + w_d

    return int(jv), int(jw)


def compute_odometry(x, y, th, rvel, lvel, delta_time):
    #convert left and right wheels velocities into v and w
    cv =  (rvel + lvel) / 2; # m/s
    cw =  (rvel - lvel) / TREAD;

    th += (cw * delta_time);
    x += cv * cos(th) * delta_time;
    y += cv * sin(th) * delta_time;

    return cv, cw, x, y, th

def name():
    '''Create a ROS node and instantiate the class.'''

    rospy.init_node('whill_driver', anonymous=True)

    port = rospy.get_param('port', '/dev/ttyUSB0')
    TREAD = rospy.get_param('tread', 0.5)

    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    x = y = th = 0.0
    motor_speed_old = Vector3()
    motor_speed_old.x = motor_speed_old.y = 0.0
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    joystick_pub = rospy.Publisher('/joystick', Vector3, queue_size=1)
    motor_pub = rospy.Publisher('/motor_speed', Vector3, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    odom = Odometry()

    whill = WHILL('/dev/ttyUSB0')
    whill.StartSendingData( 1, 100, 1)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    write_to_port_last_time = rospy.Time.now()
    rate = rospy.Rate( 100 )

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()


        dt = (current_time - last_time).to_sec()

        joy_front, joy_side, battery_power, right_motor_speed, left_motor_speed, power_on, error = whill.GetData()
        #joy_front = joy_side= battery_power= right_motor_speed= left_motor_speed= power_on= error = 0
	
	
	#modify motor speed
	
	if abs(right_motor_speed - motor_speed_old.x) > 0.5:
	    right_motor_speed = motor_speed_old.x
        motor_speed_old.x = right_motor_speed

	if abs(left_motor_speed - motor_speed_old.y) > 0.5:
	    left_motor_speed = motor_speed_old.y
	motor_speed_old.y = left_motor_speed
	    
		
	
        if joy_front != 0 or joy_side !=0:
            WHILL_JOYSTICK_ALLOWED = True
        else:
            WHILL_JOYSTICK_ALLOWED = False

        # compute odometry in a typical way given the velocities of the robot
        cv, cw, x, y, th = compute_odometry(x, y, th, right_motor_speed, left_motor_speed, dt)

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        joystick_msg = Vector3()
        joystick_msg.x = joy_front
        joystick_msg.y = joy_side

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform( (x, y, 0.), odom_quat, current_time, "base_link", "odom" )
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(cv, 0, 0), Vector3(0, 0, cw))

	# set the motor speed
	motor_speed = Vector3()
	motor_speed.x = right_motor_speed
	motor_speed.y = left_motor_speed


        odom_pub.publish(odom)
        joystick_pub.publish(joystick_msg)
	motor_pub.publish(motor_speed)

	
	

        #Only writing to port every 20msec
        write_to_port_time = (current_time - write_to_port_last_time).to_sec()
        if  write_to_port_time >= 0.02:
            # dv:velocity is in m/sec [-0.5:1.6];  dw:angular velocity rad/sec [-1:1];   joystick commands [100:-100]
            jv, jw = set_velocity_to_joystick_range( dv, dw )
            if WHILL_JOYSTICK_ALLOWED == True:
                whill.SetJoystick(1, 0, 0)
            else:
                whill.SetJoystick(0, jv, jw)
            write_to_port_last_time = current_time
            #whill.SetJoystick(0, int(dv*100), int(dw*-100))
            rospy.sleep(0.001)


        last_time = current_time
        rate.sleep()
        #print dt, joy_front, joy_side, battery_power, right_motor_speed, left_motor_speed, power_on, error, dt, x, y, th, cv, cw



if __name__ == "__main__":
    name()
