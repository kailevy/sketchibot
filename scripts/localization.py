#!/usr/bin/env python

import rospy
import tf
import numpy as np
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

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
		self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

		# Gets current position of the Neato
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.position_callback)
		rospy.Subscriber('/odom', Odometry, self.odom_callback)

		# Transformation from map to base_link
		listener = tf.TransformListener()
		listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(3.0))
		(pos, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
		initial_pose = convert_to_pose(pos, rot)

		# Initial, current, and final state variables of the Neato
		self.x_0, self.y_0, self.th_0 = convert_pose_to_xy_and_theta(initial_pose)
		self.x,   self.y,   self.th   = (0, 0, 0)
		self.x_f, self.y_f, self.th_f = (0, 0, 0)

		self.waypoints = (((0.5, 0.0, -np.pi/2), (0.5, 0.5, -np.pi/2), (0, 0.5, -np.pi/2), (0.0, 0.0, -np.pi/2)),
			              ((1.0, 0.0, np.pi/2), (1.0, 0.0, np.pi/2), (1.0, 0.0, np.pi/2), (1.0, 0.0, np.pi/2)))
		self.waypoint = 0  # Current waypoint
		self.contour = 0   # Current contour

	""" Callback function for Neato's current position """
	def position_callback(self, msg):
		pose = msg.pose.pose
		self.x, self.y, self.th = convert_pose_to_xy_and_theta(pose)

	""" Callback function for Neato's odometry reading """
	def odom_callback(self, msg):
		pose = msg.pose.pose
		# self.x, self.y, self.th = convert_pose_to_xy_and_theta(pose)

	""" Publishes a waypoint to the Neato, with the map as the coordinate frame
			pos - delta in translational motion
			rot - delta in heading """
	def push_waypoint(self, pos, rot):
		x, y = pos

		# Calculates the goal based on the 
		self.x_f = self.x_0 + x*np.cos(self.th_0) + y*np.cos(self.th_0 + np.pi/2)
		self.y_f = self.y_0 + x*np.sin(self.th_0) + y*np.sin(self.th_0 + np.pi/2)
		self.th_f = self.th_0 + rot

		# Encodes published data into a PoseStamped message
		pose = Pose(position=Point(x=self.x_f, y=self.y_f, z=0),
					orientation=Quaternion(*quaternion_from_euler(0,0,self.th_f)))
		header = Header(stamp=rospy.Time.now(), frame_id='/map')
		msg = PoseStamped(header=header, pose=pose)

		self.pub_goal.publish(msg)

	""" Checks if goal has been reached """
	def reached_goal(self):
		diff_th = self.th_f - self.th

		# Tries different multiples of 2 pi to get the correct error value (ex. 6.28 rad and 0 rad should mean zero error)
		dist1 = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2 + (diff_th)**2)
		dist2 = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2 + (diff_th - 2*np.pi)**2)
		dist3 = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2 + (diff_th + 2*np.pi)**2)

		return dist1 < 0.2 or dist2 < 0.2 or dist3 < 0.2

	""" Main loop that sends velocity commands to the Neato """
	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			i = self.contour
			j = self.waypoint

			pos = self.waypoints[i][j][0:2]
			rot = self.waypoints[i][j][2]

			self.push_waypoint(pos, rot)
			done = self.reached_goal()

			if done:
				self.waypoint += 1
				self.th_0 = self.th_f
			if self.waypoint == 4:
				print "done!"
				rospy.spin()

if __name__ == '__main__':
	node = Sketchibot()
	node.run()


""" Open the following files:
		/neato_2dnav/launch/includes/my_amcl.launch.xml
		/neato_2dnav/launch/includes/amcl.launch.xml

	Change the following parameters:
		update_min_d = 0.02
		update_min_a = 0.05
"""