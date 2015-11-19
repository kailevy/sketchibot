#!/usr/bin/env python

""" A simple script to visualize the SIFT descriptor for an interactively drawn sketch """

import numpy as np
import cv2
from math import pi, cos, sin, sqrt, atan2
import rospy

class PathDrawing():
    def __init__(self):
        self.stroke = []
        self.strokes = []
        self.drawing = False
        self.scale = 200.0

    def scale_patch(self,x,y):
        """X and Y are in feet, scale is 1 meter to 200 pixels"""
        self.pagex = float(x)
        self.pagey = float(y)
        self.patch_size = (self.pagey*self.scale,self.pagex*self.scale)
        # we will draw our patch on im
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
            #strokes.append('NNNEEEEEEEEEWWWWW STROOKKEEE')
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            # draw a line between the last mouse position and the current one
            cv2.line(self.im1,(int(x),int(y)),(int(last_x),int(last_y)),0,2)
            last_x = x
            last_y = y
            self.stroke.append([x,y])

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
                #self.im1 =  255*np.ones(self.patch_size,dtype=np.uint8)
                print self.strokes
                self.scale_strokes()
                print 'Scaled:'
                print self.strokes
                print 'Filtered:'
                self.point_distance_filtering()
                self.colinear_points()
                print self.strokes
                self.plot_strokes()
                self.strokes = []

    def scale_strokes(self):
        """scales the strokes to the page size"""
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]/self.scale
                path[i][1] = path[i][1]/self.scale

    def point_distance_filtering(self):
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    if i>1:
                        distance = sqrt((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)
                        if distance < .5:
                            path.pop(i+1)

    def colinear_points(self):
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-2:
                    if i > 2:
                        radangle1 = atan2(path[i+1][1]-path[i][1],path[i+1][0]-path[i][0])
                        radangle2 = atan2(path[i+2][1]-path[i+1][1],path[i+2][0]-path[i+1][0])
                        angle1 = radangle1*180/pi
                        angle2 = radangle2*180/pi
                        if abs(angle2-angle1) < 10:
                            path.pop(i+1)

    def add_heading(self):
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    heading = atan2(path[i+1][1],path[i+1][0])#not sure which reference frame heading is
                    path[i].append(heading)
                else:
                    path[i].append(heading)#appends previous heading if no extra path

    def plot_strokes(self):
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    cv2.arrowedLine(self.im2, (int(path[i][0]*self.scale), int(path[i][1]*self.scale)), (int(path[i+1][0]*self.scale), int(path[i+1][1]*self.scale)), (0, 0, 255), thickness=1)

if __name__ == '__main__':
    drawing = PathDrawing()
    drawing.scale_patch(2,2)
    drawing.draw_strokes()