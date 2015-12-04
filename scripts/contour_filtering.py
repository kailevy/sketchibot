#!/usr/bin/env python

"""a script to filter contours into waypoints and scale them to the paper size the neato is drawing on"""

import numpy as np
import cv2
from math import pi, cos, sin, sqrt, atan2
import rospy
from edge_detect import EdgeDetector

class ContourFiltering():
    def __init__(self, strokes = [],imsize=(1,1),pagesize=(1,1)):
        self.imx = imsize[0]
        self.imy = imsize[1]
        self.pagex= pagesize[0]
        self.pagey = pagesize[1]
        self.strokes = strokes
        imrat = self.imx/self.imy
        pagerat = self.pagex/self.pagey
        if imrat > pagerat:
            self.scalefactor = (self.pagey/self.imx)*.9
        else:
            self.scalefactor = (self.pagex/self.imy)*.9
        print self.scalefactor

        self.screenscale = 200.0
        self.patch_size = (self.pagey*self.screenscale,self.pagex*self.screenscale)
        self.im = 255*np.ones(self.patch_size,dtype=np.uint8)

    def run_filter(self):
        #scale up contours
        print 'Original'
        print self.strokes
        self.scale_strokes()
        #filter contours
        print 'Scaled'
        print self.strokes
        #self.point_filtering()
        #center contours
        self.center_contours()
        print 'Centered'
        print self.strokes
        self.plot_contours()

    def scale_strokes(self):
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]*self.scalefactor
                path[i][1] = path[i][1]*self.scalefactor

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

    def center_contours(self):
        for path in self.strokes:
            for i in range(len(path)):
                path[i][0] = path[i][0]+(self.pagex/2.0)-((self.imx/2.0)*self.scalefactor)
                path[i][1] = path[i][1]+(self.pagey/2.0)-((self.imy/2.0)*self.scalefactor)

    def plot_contours(self):
        cv2.namedWindow("Filtered Contour Plot")
        while not rospy.is_shutdown():
            cv2.imshow("Filtered Contour Plot", self.im)
            key = cv2.waitKey(5)
            for path in self.strokes:
                for i in range(len(path)):
                    if 0<i<len(path):
                        cv2.line(self.im,(int(path[i-1][0]*self.screenscale),int(path[i-1][1]*self.screenscale)),(int(int(path[i][0]*self.screenscale)),int(path[i][1]*self.screenscale)),0,2)

if __name__ == '__main__':
    contours = [[[0,0],[0,1],[.5,1],[.5,0],[0,0]]]
    drawing = ContourFiltering(strokes = contours,imsize=(.5,1),pagesize=(2,2))
    drawing.run_filter()