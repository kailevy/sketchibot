#!/usr/bin/env python

'''Drives the robot in a square using the /odom and /base_link reference frames'''

import tty
import select
import sys
import termios
import time
import rospy
import tf
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 

rospy.init_node('square_node')

tf_listener = tf.TransformListener()
tf_br = tf.TransformBroadcaster()

pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
pub2=rospy.Publisher('/servo_command',String,queue_size=10)

def line():
	now = rospy.Time.now()
	tf_listener.waitForTransform("/odom","/base_link",now+rospy.Duration(1), rospy.Duration(5.0)) #sets the reference frame to where the robot is (it's starting position)
	(start_trans,start_rot)=tf_listener.lookupTransform("/odom","/base_link",now) #sets up the "zero" translation and rotation based on the robot's reference frame
	pub.publish(forward())
	not_there=True #not_there is true when the robot has not gone a meter yet, and false once it has
	r=rospy.Rate(10)
	while not_there:
		print 'making line'
		now = rospy.Time.now()
		tf_br.sendTransform(start_trans, start_rot, now, "start","odom") #sends ROS the frame of where it started
		tf_listener.waitForTransform("/base_link","/start",now,rospy.Duration(5.0)) #transform of fram of where it is and frame of where it started
		(trans,rot) = tf_listener.lookupTransform("/base_link","/start",now)
		if math.sqrt(trans[0]**2+trans[1]**2)>1: #If total robot distance traveled is 1, finish line
			not_there=False
			return 'done'
		r.sleep()

def rotate():
	now = rospy.Time.now()
	tf_listener.waitForTransform("/odom","/base_link",now+rospy.Duration(1), rospy.Duration(5.0)) #sets the reference frame to where the robot is (it's starting position)
	(start_trans,start_rot)=tf_listener.lookupTransform("/odom","/base_link",now) #sets up the "zero" translation and rotation based on the robot's reference frame
	pub.publish(turn_right())
	not_there=True #not_there is true when the robot has not turned 90 degrees yet, and false once it has
	r=rospy.Rate(10)
	while not_there:
		print 'turning'
		now = rospy.Time.now()
		tf_br.sendTransform(start_trans, start_rot, now, "start","odom") #sends ROS the frame of where it started
		tf_listener.waitForTransform("/base_link","/start",now,rospy.Duration(5.0)) #transform of fram of where it is and frame of where it started
		(trans,rot) = tf_listener.lookupTransform("/base_link","/start",now)
		if rot[2]>0.45: 	#If turned about 90 degrees, stop
			return 'done'
		r.sleep()

#drives in a line, then turns 4 times

def square():
	# for i in range(10):
	# 	pub2.publish(String(data='1'))
	time.sleep(3)
	for i in range(4):
		time.sleep(.5)
		res=line()
		print res
		pub.publish(stop())
		res=rotate()
		print res
		pub.publish(stop())
	pub2.publish(String(data='0'))
	print 'mission complete'

#Move forward commands

forward_lin=Vector3(x=0.5,y=0.0,z=0.0)
forward_ang=Vector3(x=0.0,y=0.0,z=0.0)
forward_msg=Twist(linear=forward_lin, angular=forward_ang)

#Move backward commands

backward_lin=Vector3(x=-0.5,y=0.0,z=0.0)
backward_ang=Vector3(x=0.0,y=0.0,z=0.0)
backward_msg=Twist(linear=backward_lin, angular=backward_ang)

#turn left commands

turn_left_lin=Vector3(x=0.0,y=0.0,z=0.0)
turn_left_ang=Vector3(x=0.0,y=0.0,z=1.0)
turn_left_msg=Twist(linear=turn_left_lin, angular=turn_left_ang)

#turn right commands

turn_right_lin=Vector3(x=0.0,y=0.0,z=0.0)
turn_right_ang=Vector3(x=0.0,y=0.0,z=-1.0)
turn_right_msg=Twist(linear=turn_right_lin, angular=turn_right_ang)

#stop commands

stop_lin=Vector3(x=0.0,y=0.0,z=0.0)
stop_ang=Vector3(x=0.0,y=0.0,z=0.0)
stop_msg=Twist(linear=stop_lin, angular=stop_ang)

def forward():	#move forward
	print 'forward'
	return forward_msg

def backward():	#move backward
	print 'backward'
	return backward_msg

def turn_left():	#turn left
	print 'turn left'
	return turn_left_msg

def turn_right():	#turn right
	print 'turn right'
	return turn_right_msg

def stop():		#stop
	print 'STHAAAPPPPP'
	return stop_msg

r = rospy.Rate(5)
i = 0
while not rospy.is_shutdown():
	pub2.publish(String(data="1"))
	r.sleep()
	i += 1
	if i == 10:
		break
square()