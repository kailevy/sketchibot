#!/usr/bin/env python

import rospy
import tf
import numpy as np
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from edge_detect import *

""" Converts a translation and rotation component to a Pose object """
def convert_to_pose(translation, rotation):
	return Pose(position=Point(x=translation[0],
							   y=translation[1],
							   z=translation[2]),
				orientation=Quaternion(x=rotation[0],
									   y=rotation[1],
									   z=rotation[2],
									   w=rotation[3]))

""" Extracts position and angle information from a Pose object """
def convert_pose_to_xy_and_theta(pose):
	orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
	angles = euler_from_quaternion(orientation_tuple)
	return (pose.position.x, pose.position.y, angles[2])

""" Main node that commands the Neato to go to waypoints, given a series of points from an image """
class Sketchibot(object):

	""" Initialzies important variables and nodes """
	def __init__(self):
		rospy.init_node('sketchibot')

		# Publishes directly to the navigation stack
		self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Gets current position of the Neato
		# rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.position_callback)
		rospy.Subscriber('/odom', Odometry, self.odom_callback)

		# # Transformation from map to base_link
		# listener = tf.TransformListener()
		# listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(3.0))
		# (pos, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
		# initial_pose = convert_to_pose(pos, rot)

		# Initial, current, and final state variables of the Neato
		self.x_0, self.y_0, self.th_0 = (0, 0, 0)
		self.x,   self.y,   self.th   = (0, 0, 0)
		self.x_f, self.y_f, self.th_f = (0, 0, 0)

		self.contours = None

	# """ Callback function for Neato's current position """
	# def position_callback(self, msg):
	# 	pose = msg.pose.pose
	#   self.x, self.y = convert_pose_to_xy_and_theta(pose)[0:2]

	""" Callback function for Neato's odometry reading """
	def odom_callback(self, msg):
		pose = msg.pose.pose
		self.x, self.y, self.th = convert_pose_to_xy_and_theta(pose)

	""" Publishes a waypoint to the Neato, with the map as the coordinate frame
			pos - delta in translational motion
			rot - delta in heading """
	def push_waypoint(self, pos, rot):
		x, y = pos  # Location of the next goal

		self.x_f, self.y_f = pos
		self.th_f = rot

		# Rotates the Neato towards the next goal
		self.rotate_neato()

		# Drives the Neato forwards until goal is reached
		self.drive_neato()

	""" Checks if goal has been reached """
	def reached_goal(self):
		dist = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2)
		return dist < 0.1

	""" Checks if the Neato has rotated towards the target goal """
	def correct_heading(self):
		return (self.th_f - self.th) < 0.02

	""" Moves the Neato until it is at the waypoint """
	def forwards_neato(self):
		while not self.reached_goal():
			xy_error = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2)
			K_xy = 1.5  # Proportional control

			vel = Twist()
			vel.linear.x = K * xy_error
			self.pub_vel.publish(vel)

	""" Rotates the Neato until its heading is correct """
	def rotate_neato(self):
		while not self.correct_heading():
			th_error = self.th_f - self.th
			K = 2  # Proportional control

			vel = Twist()
			vel.angular.z = K * th_error
			self.pub_vel.publish(vel)

	""" Calculates angle between two points """
	def calc_theta(self, x1, y1, x2, y2):
		return math.atan2(y1-y2, x1-x2)

	""" Gets a list of points to follow """
	def get_contours(self):
		return [[np.array([0, 1], dtype=np.int32), np.array([1, 1], dtype=np.int32)]]

	""" Main loop that sends velocity commands to the Neato """
	def run(self):
		self.contours = self.get_contours()
		done = False
		prev = [0, 0]

		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			for i in self.contours:
				for j in i:	
					pos = [j[0], j[1]]
					rot = self.calc_theta(prev[0], prev[1], j[0], j[1])

					while not done:

						self.push_waypoint(pos, rot)
						done = self.reached_goal()

						if done:
							self.th_0 = self.th_f
							prev = pos

if __name__ == '__main__':
	node = Sketchibot()
	node.run()