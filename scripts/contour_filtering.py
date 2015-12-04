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
        if imrat < pagerat:
            self.scalefactor = (self.pagey/self.imx)*.9
        else:
            self.scalefactor = (self.pagex/self.imy)*.9
        print self.scalefactor

    def run_filter(self):
    	#scale up contours
    	self.scale_strokes()
    	#filter contours
    	#center contours

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

if __name__ == '__main__':
    contours = [[[0,0],[0,1],[.5,1],[.5,0]]]
    drawing = ContourFiltering(strokes = contours,imsize=(1,1),pagesize=(2,2))
    drawing.run_filter()