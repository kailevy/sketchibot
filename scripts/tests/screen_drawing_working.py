#!/usr/bin/env python

""" A simple script to test Neato drawing and waypoint filtering """

import numpy as np
import cv2
from math import pi, cos, sin, sqrt, atan2
import rospy

class PathDrawing():
    """A class that allows a path to be drawn, filtered, and sent to the Neato"""
    def __init__(self):
        self.stroke = []
        self.strokes = []
        self.drawing = False
        self.scale = 200.0 #scaling factor
        self.barcount = 0

    def scale_patch(self,x,y):
        """X and Y are in feet, scale is 1 meter to 200 pixels"""
        self.pagex = float(x)
        self.pagey = float(y)
        self.patch_size = (self.pagey*self.scale,self.pagex*self.scale) #patch sized scales to page size
        # im1 is the drawing patch, im2 is the patch waypoints are drawn on
        self.im1 = 255*np.ones(self.patch_size,dtype=np.uint8)
        self.im2 = 255*np.ones(self.patch_size,dtype=np.uint8)

    def mouse_event(self,event,x,y,flag,dc):
        """ handle mouse events, basically lets you sketch by clicking in the left pane """
        global last_x
        global last_y
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            last_x = x
            last_y = y
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.strokes.append(self.stroke)
            self.stroke = []
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            # draw a line between the last mouse position and the current one
            cv2.line(self.im1,(int(x),int(y)),(int(last_x),int(last_y)),0,2)
            last_x = x
            last_y = y
            self.stroke.append([x,y]) #adds recent waypoint to stroke

    def draw_strokes(self):
        cv2.namedWindow("mywin1")
        cv2.namedWindow("mywin2")
        cv2.setMouseCallback("mywin1", self.mouse_event)

        print "Draw on the canvas by clicking and holding the mouse (move slowly)"
        print "Reset the sketch by pressing the spacebar"

        while not rospy.is_shutdown():
            cv2.imshow("mywin1", self.im1)
            cv2.imshow("mywin2", self.im2)
            key = cv2.waitKey(25)
            if key != -1 and chr(key) == ' ':
                # if you hit space bar, you should reset the sketch on the left
                #print self.strokes
                self.scale_strokes()
                #print 'Scaled:'
                print self.strokes
                print 'Filtered:'
                self.point_filtering()
                print self.strokes
                self.plot_strokes()
                self.strokes = []
                
    def scale_strokes(self):
        """scales the strokes to the page size"""
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]/self.scale
                path[i][1] = path[i][1]/self.scale

    def point_filtering(self):
        """filters all of the waypoints down to certain distance and angle margins"""
        filtered_cpaths = [] #creates new list of filtered paths
        threshold = .2 #minimum distance between two points
        angle_thresh = 20 #minimum angle between three points
        for path in self.strokes:
            filtered_cpath = [] #creates new filtered path
            i = 0 #index only of points above threshold criteria
            filtered_cpath.append(path[-1])
            while i < len(path) - 3:
                #only appends points to path above threshold criteria
                j = i + 1 #j loops through all points
                filtered_cpath.append(path[i])
                distance = 0
                anglediff = 0.0
                #i only increments when function finds values above threshold
                while distance < threshold and anglediff < angle_thresh and j < len(path)-2: #while 3 points are below threshold
                    radangle1 = atan2(path[j][1]-path[i][1],path[j][0]-path[i][0]) #angle between first 2 points and 0
                    radangle2 = atan2(path[j+1][1]-path[j][1],path[j+1][0]-path[j][0]) #angle between second 2 points and 0
                    #convert angles to degrees
                    angle1 = radangle1*180.0/pi
                    angle2 = radangle2*180.0/pi
                    anglediff = abs(angle2-angle1) #calculates difference between angles
                    distance = sqrt((path[i][0]-path[j][0])**2 + (path[i][1]-path[j][1])**2) #calculates distance between two points
                    j += 1 #indexes j values
                i = j #starts i at next j value in list
            filtered_cpaths.append(filtered_cpath) #adds filtered path to paths
        self.strokes = filtered_cpaths  #resets self.strokes

    def add_heading(self):
        """adds heading to each stroke if necessary"""
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    heading = atan2(path[i+1][1],path[i+1][0])#not sure which reference frame heading is
                    path[i].append(heading)
                else:
                    path[i].append(heading) #appends previous heading if no extra path

    def plot_strokes(self):
        """plots all the strokes after fintering"""
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    cv2.circle(self.im2, (int(path[i][0]*self.scale), int(path[i][1]*self.scale)), 1,(0, 0, 255), thickness=1)

if __name__ == '__main__':
    drawing = PathDrawing()
    drawing.scale_patch(2,2)
    drawing.draw_strokes()
