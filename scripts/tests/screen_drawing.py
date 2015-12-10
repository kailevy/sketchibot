#!/usr/bin/env python

"""a script to filter contours into waypoints and scale them to the paper size the neato is drawing on"""

import numpy as np
import cv2
from math import pi, cos, sin, sqrt, atan2
import rospy
from edge_detect import EdgeDetector

class PathDrawing():
    def __init__(self, strokes = [],imsize=(1,1),pagesize=(1,1)):
        self.xlim = imsize[0]
        self.ylim = imsize[1]
        self.pagex= pagesize[0]
        self.pagey = pagesize[1]
        self.stroke = []
        self.strokes = strokes
        self.drawing = False
        self.scale = 200.0

    def run_waypoints(self):
        self.scale_patch()
        self.scale_calc()
        #plot existing strokes
        self.scale_strokes()
        print self.strokes
        self.plot_existing_strokes()

        #draw things

        self.draw_strokes()
        if key != -1 and chr(key) == ' ':
            self.point_filtering()
        #filter and scale everything
        #prompt "press space when satisfied"
        #pressing space clears the patch and returns self.strokes
        #strokes is reset

    def scale_patch(self):
        """X and Y are in feet, scale is 1 meter to 200 pixels"""
        self.patch_size = (self.pagey*self.scale,self.pagex*self.scale)
        # we will draw our patch on im
        self.im1 = 255*np.ones(self.patch_size,dtype=np.uint8)
        self.im2 = 255*np.ones(self.patch_size,dtype=np.uint8)

    def plot_existing_strokes(self):
        for path in self.strokes:
            for i in range(len(path)):
                if 0<i<len(path):
                    cv2.line(self.im1,(int(path[i-1][0]),int(path[i-1][1])),(int(int(path[i][0])),int(path[i][1])),0,2)

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
            self.stroke.append([x,y])

    def draw_strokes(self):
        cv2.namedWindow("mywin1")
        cv2.namedWindow("mywin2")
        cv2.setMouseCallback("mywin1", self.mouse_event)

        print "Draw on the canvas by clicking and holding the mouse (move slowly)"
        print "Once you are satisfied, press the spacebar"

        while not rospy.is_shutdown():
            cv2.imshow("mywin1", self.im1)
            cv2.imshow("mywin2", self.im2)
            key = cv2.waitKey(25)
            if key != -1 and chr(key) == ' ':
                # if you hit space bar, you should reset the sketch on the left
                self.im1 =  255*np.ones(self.patch_size,dtype=np.uint8)
                #self.strokes = []
                print self.strokes
                self.scale_strokes2()
                #print 'Scaled:'
                #print self.strokes
                #print 'Filtered:'

                print 'Filtered'
                print self.strokes
                self.plot_waypoints()

    #def scale_filter_plot(self):
    def scale_calc(self):
        imrat = self.xlim/self.ylim
        pagerat = self.pagex/self.pagey
        if imrat < pagerat:
            self.scalefactor = (self.pagey/self.ylim)*.9
        else:
            self.scalefactor = (self.pagex/self.xlim)*.9

    def scale_strokes(self):
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]*self.scalefactor
                path[i][1] = path[i][1]*self.scalefactor

    def scale_strokes2(self):
      """scales the strokes to the page size"""
     for path in self.strokes:
        for i in range(len(path)):
           path[i][0] = path[i][0]/self.scale
          path[i][1] = path[i][1]/self.scale

    def point_filtering(self):
        filtered_cpaths = []
        threshold = .2
        angle_thresh = 20
        for path in self.strokes:
            filtered_cpath = []
            i = 0
            while i < len(path) - 3:
                j = i + 1
                filtered_cpath.append(path[i])
                distance = 0
                anglediff = 0
                while distance < threshold and anglediff < angle_thresh and j < len(path)-2:
                    radangle1 = atan2(path[j][1]-path[i][1],path[j][0]-path[i][0])
                    radangle2 = atan2(path[j+1][1]-path[j][1],path[j+1][0]-path[j][0])
                    angle1 = radangle1*180/pi
                    angle2 = radangle2*180/pi
                    anglediff = abs(angle2-angle1)
                    distance = sqrt((path[i][0]-path[j][0])**2 + (path[i][1]-path[j][1])**2)
                    j += 1
                i = j
            filtered_cpath.append(path[-1])
            filtered_cpaths.append(filtered_cpath)
        self.strokes = filtered_cpaths  

    def add_heading(self):
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    heading = atan2(path[i+1][1],path[i+1][0])#not sure which reference frame heading is
                    path[i].append(heading)
                else:
                    path[i].append(heading)#appends previous heading if no extra path

    def plot_waypoints(self):
        for path in self.strokes:
            for i in range(len(path)):
                if i<len(path)-1:
                    cv2.circle(self.im2, (int(path[i][0]/self.scalefactor), int(path[i][1]/self.scalefactor)), 1,(0, 0, 255), thickness=1)

if __name__ == '__main__':
    contours = [[[0,0],[0,1],[.5,1],[.5,0]]]
    drawing = PathDrawing(strokes = contours,imsize=(1,1),pagesize=(2,2))
    drawing.run_waypoints()