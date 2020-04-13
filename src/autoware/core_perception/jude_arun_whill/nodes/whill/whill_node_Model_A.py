#!/usr/bin/env python

#Created on Monday May 14th 2018
#author: Luis Yoichi Morales
#chaged by: Shunichiro Sugiyama
#import roslib; roslib.load_manifest('whill_driver')
import rospy
import time
import std_msgs.msg
import tf
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from whill.msg import whill_data
from math import sin, cos, pi

from sensor_msgs.msg import *
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import*
from time import sleep

#geometry_msgs::Twist::ConstPtr

from WHILL_Command_Model_A import WHILL
from WHILL_Command_Model_A import to_bytes
#BASELINE = 0.5


dv = 0
dw = 0
cmd_vel_buffer = Twist()
cmd_vel_time = 0

def cmd_callback(cmd):
    global cmd_vel_buffer
    global cmd_vel_time
    cmd_vel_buffer = cmd
    cmd_vel_time = rospy.Time.now()
#Transforms velocity commands to joystick commands (values extracted with a person weighting 70Kg)
#dv:velocity is in m/sec [-0.5:1.6];  dw:angular velocity rad/sec [-1:1];   joystick commands [100:-100]
#Parameters a,b,c,_ud were computed using nonlinear least-squares (NLLS) Marquardt-Levenberg algorithm.
def set_velocity_to_joystick_range( dv, dw ):
    jv = dv * 150
    jw = -dw * 23

    jv = min(jv, 100)
    jv = max(jv, -100)
    jw = min(jw, 100)
    jw = max(jw, -100)

    return int(jv), int(jw)


#def compute_odometry(x, y, th, rvel, lvel, delta_time):
#    #convert left and right wheels velocities into v and w
#    cv =  (rvel + lvel) / 2; # m/s
#    cw =  (rvel - lvel) / BASELINE;
#
#    th += (cw * delta_time);
#    x += cv * cos(th) * delta_time;
#    y += cv * sin(th) * delta_time;
#
#    return cv, cw, x, y, th

def name():
    '''Create a ROS node and instantiate the class.'''

    rospy.init_node('whill_driver', anonymous=True)

    port = rospy.get_param('~port', '/dev/whill/whill')
    drive_mode = rospy.get_param("~drive_mode", "normal")

    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    x = y = th = 0.0
    whill_pub = rospy.Publisher('/whill_data', whill_data, queue_size=1)
    joystick_pub = rospy.Publisher('~/joystick', Vector3, queue_size=1)
    speed_pub = rospy.Publisher('/raw_speed', Float32, queue_size=1)
    speed = Float32()

    print ("Opening WHILL at port:" + str(port) + "\n\n\n")

    whill = WHILL( port )

    if drive_mode == "high_speed":
        whill.SetForward(80,10,100);  whill.SetReverse(30,10,40);  whill.SetTurn(60,40,70)
        print ("drive_mode: high_speed")
    elif drive_mode == "maximum":
        whill.SetForward(100,100,100);  whill.SetReverse(100,100,100);  whill.SetTurn(100,50,100);
        print ("drive_mode: maximum")
    else:
        ''' default '''
        whill.SetForward(29,25,60);  whill.SetReverse(11,11,11);  whill.SetTurn(50,50,50);
        print ("drive_mode: default")


    #whill.StartSendingProfile(10)
    #whill.GetProfile()
    #sleep(1)


    whill.StartSendingData(100)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    write_to_port_last_time = rospy.Time.now()
    rate = rospy.Rate( 100 )

    global cmd_vel_time
    cmd_vel_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()


        #dt = (current_time - last_time).to_sec()

        acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, joy_front, joy_side, battery_power, speed, power_on, error, id_position = whill.GetData()
        #joy_front = joy_side= battery_power= right_motor_speed= left_motor_speed= power_on= error = 0

        if error > 0:
            continue

        if joy_front != 0 or joy_side != 0:
            DRIVER_OVERWRAPPED = True
        else:
            DRIVER_OVERWRAPPED = False

        whill_raw_data = whill_data()
        whill_raw_data.header.stamp = rospy.Time.now()
        whill_raw_data.header.frame_id = "whill"
        whill_raw_data.acc_x = acc_x
        whill_raw_data.acc_y = acc_y
        whill_raw_data.acc_z = acc_z
        whill_raw_data.gyr_x = gyr_x
        whill_raw_data.gyr_y = gyr_y
        whill_raw_data.gyr_z = gyr_z
        whill_raw_data.joy_front = joy_front
        whill_raw_data.joy_side = joy_side
        whill_raw_data.battery_power = battery_power
        whill_raw_data.speed = speed
        whill_raw_data.power_on = power_on
        whill_raw_data.error = error
        whill_raw_data.id_position = id_position

        # compute odometry in a typical way given the velocities of the robot
        right_motor_speed = 0
        left_motor_speed = 0
        #cv, cw, x, y, th = compute_odometry(x, y, th, right_motor_speed, left_motor_speed, dt)

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        #odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        joystick_msg = Vector3()
        joystick_msg.x = joy_front
        joystick_msg.y = joy_side

        # first, we'll publish the transform over tf
        #odom_broadcaster.sendTransform( (x, y, 0.), odom_quat, current_time, "base_link", "odom" )
        #odom.header.stamp = rospy.Time.now()
        #odom.header.frame_id = "odom"

        # set the position
        #odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        #odom.child_frame_id = "base_link"
        #odom.twist.twist = Twist(Vector3(cv, 0, 0), Vector3(0, 0, cw))

        #odom_pub.publish(odom)
        whill_pub.publish(whill_raw_data)
        joystick_pub.publish(joystick_msg)
        speed_pub.publish(speed)

        #Only writing to port every 20msec
        write_to_port_time = (current_time - write_to_port_last_time).to_sec()
        if  write_to_port_time >= 0.02:
            # dv:velocity is in m/sec [-0.5:1.6];  dw:angular velocity rad/sec [-1:1];   joystick commands [100:-100]
            if DRIVER_OVERWRAPPED == False:

                if (current_time - cmd_vel_time).to_sec() < 0.2:
                    jv, jw = set_velocity_to_joystick_range( cmd_vel_buffer.linear.x, cmd_vel_buffer.angular.z )
                    whill.SetJoystick(0, jv, jw)

            write_to_port_last_time = current_time
            #whill.SetJoystick(0, int(dv*100), int(dw*-100))
            rospy.sleep(0.001)


        last_time = current_time
        rate.sleep()
        #print dt, joy_front, joy_side, battery_power, right_motor_speed, left_motor_speed, power_on, error, dt, x, y, th, cv, cw



if __name__ == "__main__":
    name()
