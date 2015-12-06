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
        self.point_filtering()
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
            filtered_cpath.append(path[-1])
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
    contours = [[[0.725, 0.97], [0.73, 0.97], [0.735, 0.97], [0.74, 0.97], [0.745, 0.97], [0.75, 0.97], [0.76, 0.97], [0.77, 0.97], [0.78, 0.97], [0.79, 0.97], [0.8, 0.97], [0.81, 0.97], [0.82, 0.97], [0.83, 0.97], [0.84, 0.97], [0.85, 0.97], [0.86, 0.97], [0.87, 0.97], [0.88, 0.97], [0.89, 0.97], [0.9, 0.97], [0.91, 0.97], [0.925, 0.97], [0.94, 0.97], [0.955, 0.97], [0.97, 0.97], [0.985, 0.97], [1.0, 0.97], [1.015, 0.97], [1.03, 0.97], [1.045, 0.97], [1.06, 0.97], [1.075, 0.97], [1.09, 0.97], [1.105, 0.97], [1.12, 0.97], [1.135, 0.97], [1.15, 0.97], [1.165, 0.97], [1.18, 0.97], [1.195, 0.97], [1.21, 0.97], [1.225, 0.97], [1.24, 0.97], [1.255, 0.97], [1.27, 0.97], [1.285, 0.97], [1.3, 0.97], [1.315, 0.97], [1.33, 0.97], [1.34, 0.97], [1.35, 0.97], [1.36, 0.97], [1.365, 0.97], [1.37, 0.97], [1.375, 0.97], [1.38, 0.97], [1.385, 0.97], [1.39, 0.97], [1.395, 0.965], [1.4, 0.96], [1.405, 0.955], [1.405, 0.95], [1.405, 0.945], [1.405, 0.94], [1.405, 0.935], [1.405, 0.925], [1.405, 0.915], [1.405, 0.905], [1.41, 0.895], [1.415, 0.885], [1.42, 0.875], [1.425, 0.865], [1.43, 0.855], [1.435, 0.845], [1.44, 0.835], [1.445, 0.825], [1.45, 0.815], [1.455, 0.805], [1.46, 0.795], [1.465, 0.785], [1.47, 0.775], [1.475, 0.765], [1.48, 0.755], [1.485, 0.745], [1.49, 0.735], [1.495, 0.725], [1.5, 0.715], [1.505, 0.705], [1.51, 0.695], [1.515, 0.685], [1.52, 0.675], [1.525, 0.665], [1.53, 0.655], [1.535, 0.645], [1.54, 0.635], [1.545, 0.625], [1.55, 0.615], [1.555, 0.605], [1.555, 0.6], [1.555, 0.595], [1.555, 0.59], [1.555, 0.585], [1.555, 0.58], [1.555, 0.575], [1.555, 0.57], [1.555, 0.565], [1.555, 0.56], [1.555, 0.555], [1.555, 0.55], [1.55, 0.545], [1.545, 0.54], [1.54, 0.535], [1.535, 0.53], [1.53, 0.525], [1.52, 0.52], [1.51, 0.515], [1.5, 0.51], [1.49, 0.505], [1.48, 0.5], [1.47, 0.495], [1.46, 0.49], [1.45, 0.485], [1.44, 0.48], [1.43, 0.475], [1.42, 0.47], [1.41, 0.465], [1.4, 0.46], [1.39, 0.455], [1.38, 0.45], [1.37, 0.445], [1.36, 0.44], [1.35, 0.435], [1.34, 0.43], [1.33, 0.425], [1.32, 0.42], [1.31, 0.415], [1.3, 0.415], [1.29, 0.415], [1.28, 0.415], [1.27, 0.415], [1.26, 0.415], [1.25, 0.415], [1.24, 0.415], [1.23, 0.415], [1.22, 0.415], [1.21, 0.415], [1.2, 0.415], [1.19, 0.415], [1.18, 0.415], [1.17, 0.415], [1.16, 0.415], [1.15, 0.415], [1.14, 0.415], [1.13, 0.415], [1.12, 0.415], [1.11, 0.415], [1.1, 0.415], [1.09, 0.415], [1.08, 0.415], [1.07, 0.415], [1.06, 0.415], [1.05, 0.415], [1.04, 0.415], [1.03, 0.415], [1.02, 0.415], [1.01, 0.415], [1.0, 0.415], [0.995, 0.415], [0.99, 0.415], [0.985, 0.415], [0.98, 0.415], [0.975, 0.42], [0.97, 0.425], [0.965, 0.43], [0.96, 0.435], [0.955, 0.44], [0.95, 0.445], [0.945, 0.45], [0.94, 0.455], [0.935, 0.46], [0.93, 0.465], [0.925, 0.47], [0.92, 0.475], [0.915, 0.48], [0.91, 0.485], [0.905, 0.49], [0.9, 0.495], [0.895, 0.5], [0.89, 0.505], [0.885, 0.51], [0.88, 0.515], [0.875, 0.52], [0.87, 0.525], [0.865, 0.53], [0.86, 0.535], [0.855, 0.54], [0.85, 0.545], [0.845, 0.55], [0.84, 0.555], [0.835, 0.56], [0.83, 0.565], [0.825, 0.57], [0.82, 0.575], [0.815, 0.58], [0.815, 0.585], [0.815, 0.59], [0.815, 0.595], [0.815, 0.6], [0.815, 0.605], [0.815, 0.61], [0.815, 0.615], [0.815, 0.62], [0.815, 0.625], [0.815, 0.63], [0.815, 0.635], [0.815, 0.64], [0.815, 0.65], [0.815, 0.66], [0.815, 0.67], [0.815, 0.68], [0.815, 0.69], [0.815, 0.7], [0.815, 0.71], [0.815, 0.72], [0.815, 0.73], [0.815, 0.74], [0.815, 0.75], [0.815, 0.76], [0.815, 0.77], [0.815, 0.78], [0.815, 0.79], [0.815, 0.8], [0.815, 0.81], [0.815, 0.82], [0.815, 0.83], [0.81, 0.84], [0.805, 0.85], [0.8, 0.86], [0.795, 0.87], [0.79, 0.88], [0.785, 0.885], [0.78, 0.89], [0.775, 0.895], [0.77, 0.9], [0.765, 0.905], [0.76, 0.91], [0.75, 0.915], [0.74, 0.92], [0.73, 0.925], [0.72, 0.93], [0.71, 0.935], [0.7, 0.94], [0.69, 0.94], [0.68, 0.94], [0.67, 0.945], [0.66, 0.95], [0.655, 0.955], [0.65, 0.96], [0.645, 0.965], [0.64, 0.97], [0.635, 0.975], [0.635, 0.98], [0.635, 0.985], [0.635, 0.99]]]
    drawing = ContourFiltering(strokes = contours,imsize=(2,2),pagesize=(2,2))
    drawing.run_filter()