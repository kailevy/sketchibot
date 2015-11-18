#!/usr/bin/env python

""" A simple script to visualize the SIFT descriptor for an interactively drawn sketch """

import numpy as np
import cv2
from math import pi, cos, sin
import rospy

class PathDrawing():
    def __init__(self):
        self.stroke = []
        self.strokes = []
        self.drawing = False
        self.scale = 100

    def scale_patch(self,x,y):
        """X and Y are in feet, scale is 1 ft to 100 pixels"""
        self.pagex = float(x)*self.scale
        self.pagey = float(y)*self.scale
        self.patch_size = (self.pagey,self.pagex)
        # we will draw our patch on im
        self.im = 255*np.ones(self.patch_size,dtype=np.uint8)

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
            cv2.line(self.im,(int(x),int(y)),(int(last_x),int(last_y)),0,2)
            last_x = x
            last_y = y
            self.stroke.append([x,y])

    def draw_strokes(self):
        cv2.namedWindow("mywin")
        cv2.setMouseCallback("mywin", self.mouse_event)

        print "Draw on the canvas by clicking and holding the mouse (move slowly)"
        print "Reset the sketch by pressing the spacebar"

        while not rospy.is_shutdown():
            cv2.imshow("mywin", self.im)
            key = cv2.waitKey(25)
            if key != -1 and chr(key) == ' ':
                # if you hit space bar, you should reset the sketch on the left
                self.im =  255*np.ones(self.patch_size,dtype=np.uint8)
                print self.strokes
                self.scale_strokes()
                print 'Normalized:'
                print self.strokes
                self.strokes = []

    def normalize_strokes(self):
        """normalizes strokes to 1"""
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]/self.pagex
                path[i][1] = path[i][1]/self.pagey

    def scale_strokes(self):
        """scales strokes to paper size. THIS PART ISNT QUITE WORKING"""
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]*(self.scale/self.pagex)
                path[i][1] = path[i][1]*(self.scale/self.pagey)       

if __name__ == '__main__':
    drawing = PathDrawing()
    drawing.scale_patch(4,4)
    drawing.draw_strokes()