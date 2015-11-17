#!/usr/bin/env python

""" A simple script to visualize the SIFT descriptor for an interactively drawn sketch """

import numpy as np
import cv2
from math import pi, cos, sin
import rospy



waypoints = []
drawing = False
V = None
patch_size = (512,512)
# we will draw our patch on im
im =  255*np.ones(patch_size,dtype=np.uint8)

def mouse_event(event,x,y,flag,dc):
	""" handle mouse events, basically lets you sketch by clicking in the left pane """
	global drawing
	global last_x
	global last_y
	if event == cv2.EVENT_LBUTTONDOWN:
		drawing = True
		last_x = x
		last_y = y
	elif event == cv2.EVENT_LBUTTONUP:
		drawing = False
	elif event == cv2.EVENT_MOUSEMOVE and drawing:
		# draw a line between the last mouse position and the current one
		cv2.line(im,(int(x),int(y)),(int(last_x),int(last_y)),0,2)
		last_x = x
		last_y = y
		waypoints.append[(x,y)]

if __name__ == '__main__':
	cv2.namedWindow("mywin")
	cv2.setMouseCallback("mywin",mouse_event)

	print "Draw on the canvas by clicking and holding the mouse (move slowly)"
	print "Reset the sketch by pressing the spacebar"

	while not rospy.is_shutdown():
		cv2.imshow("mywin", im)
		key = cv2.waitKey(25)
		if key != -1 and chr(key) == ' ':
			# if you hit space bar, you should resest the sketch on the left
			im =  255*np.ones(patch_size,dtype=np.uint8)
			print waypoints