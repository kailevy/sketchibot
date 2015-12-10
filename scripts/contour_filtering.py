#!/usr/bin/env python

"""a script to filter an array of contours into waypoints and scale them to the paper size the neato is drawing on"""

import numpy as np
import cv2
from math import pi, cos, sin, sqrt, atan2
import rospy
from edge_detect import EdgeDetector

class ContourFiltering():
    """Inputs an array of points from a contour, image size, and size of the paper
    contains functions that output a filtered array of waypoints scaled to the appropriate page size"""
    def __init__(self, strokes = [],imsize=(1,1),pagesize=(2,2)):
        """inputs should be from Edge Detector
        strokes: list of strokes, where each stroke is a list of (x,y) points from edge detection
        imsize: size of the image used to make the strokes
        page size: size of physical paper being drawn on, in meters"""
        #image size values
        self.imx = float(imsize[0])
        self.imy = float(imsize[1])
        # page size values
        self.pagex= float(pagesize[0])
        self.pagey = float(pagesize[1])
        #strokes
        self.strokes = strokes
        #image aspect ratio
        imrat = self.imx/self.imy
        #paper aspect ratio
        pagerat = self.pagex/self.pagey

        """Determines scale factor to scale up image waypoints 
        so that the image will fit inside the page no matter which side is bigger"""
        if imrat < pagerat:
        #if the image ratio is smaller than the page ratio, scale to height of page, with some wiggle room
            self.scalefactor = (self.pagey/self.imx)*.75
        else:
         #if the image ratio is smaller than the page ratio, scale to width of page, with some wiggle room
            self.scalefactor = (self.pagex/self.imy)*.75

        #print self.scalefactor

        self.screenscale = 200.0 #scale factor for dislaying contours on screen--200 pixels to 1 meter
        self.patch_size = (self.pagey*self.screenscale,self.pagex*self.screenscale) #scales window to page size ratio

    def run_filter(self):
        """Runs all the filtering, scaling, and moving of strokes in the correct order"""
        self.scale_strokes() #scale up contours to correct size
        self.point_filtering() #filter contours
        self.center_contours() #center contours in page
        #self.remove_paths() #removes short strokes

    def scale_strokes(self):
        """Scales waypoints in original strokes to paper size"""
        for path in self.strokes:
            #loops through paths
            for i in range(len(path)):
                #for every point, multiplies point coordinates by scalefactor
                path[i][0] = path[i][0]*self.scalefactor
                path[i][1] = path[i][1]*self.scalefactor

    def point_filtering(self):
        """filters points based on angle and distance thresholds"""
        filtered_cpaths = [] #creates new list of filtered paths
        threshold = .2  #minimum distance between two points
        angle_thresh = 30.0 #minimum angle between three points
        for path in self.strokes:
            #loops through paths and filters them
            filtered_cpath = [] #creates new filtered path
            i = 0 #index only of points above threshold criteria
            while i < len(path) - 3:
                #only appends points to path above threshold criteria
                j = i + 1   #j loops through all points
                filtered_cpath.append(path[i]) #only appends i values to filtered list
                distance = 0 #distance between two points
                anglediff = 0 #angle made by three points
                #i only increments when function finds values above threshold
                while distance < threshold and anglediff < angle_thresh and j < len(path)-2: #while 3 points are below threshold
                    radangle1 = atan2(path[j][1]-path[i][1],path[j][0]-path[i][0])  #angle between first 2 points and 0
                    radangle2 = atan2(path[j+1][1]-path[j][1],path[j+1][0]-path[j][0]) #angle between second 2 points and 0
                    #convert angles to degrees
                    angle1 = radangle1*180.0/pi
                    angle2 = radangle2*180.0/pi
                    anglediff = abs(angle2-angle1) #calculates difference between angles
                    distance = sqrt((path[i][0]-path[j][0])**2 + (path[i][1]-path[j][1])**2) #calculates distance between two points
                    j += 1 #indexes j values
                i = j #starts i at next j value in list
            filtered_cpath.append(path[-1]) #appends last point in path to the list
            filtered_cpaths.append(filtered_cpath) #append filtered path to list of paths
        self.strokes = filtered_cpaths #sets self.strokes to filtered path

    def center_contours(self):
        """centers contours in the middle of the page"""
        for path in self.strokes:
            #loops through all paths
            for i in range(len(path)):
                #for a point in a path, adds half the page size and subtracts half the image size times the scale factor to center the image
                path[i][0] = path[i][0]+(self.pagex/2.0)-((self.imx/2.0)*self.scalefactor)
                path[i][1] = path[i][1]+(self.pagey/2.0)-((self.imy/2.0)*self.scalefactor)

    def plot_contours(self):
        """plots the paths in a window"""
        self.im = 255*np.ones(self.patch_size,dtype=np.uint8)   #makes window all white
        cv2.namedWindow("Filtered Contour Plot") #names the window
        while not rospy.is_shutdown():
            cv2.imshow("Filtered Contour Plot", self.im) #displays window
            key = cv2.waitKey(5)
            for path in self.strokes:
                #loops through all paths
                for i in range(len(path)):
                    #plots a line from one point in a path to the next
                    if 0<i<len(path):
                        cv2.line(self.im,(int(path[i-1][0]*self.screenscale),int(path[i-1][1]*self.screenscale)),(int(int(path[i][0]*self.screenscale)),int(path[i][1]*self.screenscale)),(0,0,0),2)

    def plot_points(self):
        """plots the waypoints in a window"""
        self.im2 = 255*np.ones(self.patch_size,dtype=np.uint8)   #makes window all white
        cv2.namedWindow("Filtered Waypoint Plot") #names the window
        while not rospy.is_shutdown():
            cv2.imshow("Filtered Waypoint Plot", self.im2) #displays window
            key = cv2.waitKey(5)
            for path in self.strokes:
                #loops through all paths
                for i in range(len(path)):
                    #plots a small circle for each point in the path
                    if 0<i<len(path):
                        cv2.circle(self.im2,(int(path[i][0]*self.screenscale),int(path[i][1]*self.screenscale)),1,(0,0,0),1)                    

    def get_strokes(self):
        """returns all the filtered strokes"""
        return self.strokes

    def get_number_of_waypoints(self):
        """calculates the number of waypoints in an image"""
        totalpts = 0.0  #total number of points
        for path in self.strokes:
            #loops through all paths
            totalpts += len(path) #sums all the path lengths
        return totalpts #returns total number of waypoints in the image

    def remove_paths(self):
        """removes short paths from all strokes"""
        shortest_stroke = 3 #shortest wanted stroke
        for path in self.strokes:
            #loops through all paths
            if len(path) < shortest_stroke:
                self.strokes.pop(self.strokes.index(path))

if __name__ == '__main__':
    #edge detection stuff
    detector = EdgeDetector(image_path="../images/cow.png") #creates edge detection class
    detector.reconstruct_contours()     #makes contours
    detector.sort_contours()            #sorts them to make the Neato's job easier
    contours = detector.get_contours()  #actually gets image contours
    size = detector.get_size()          #gets size of image
    drawing = ContourFiltering(strokes = contours,imsize=size) #creates contour filtering class 
    drawing.run_filter()                #runs filtering and centering methods on contours
    waypts = drawing.get_number_of_waypoints()  #gets number of waypoints
    print waypts                                #prints number of waypoints
    strokes = drawing.get_strokes()             #returns strokes
    drawing.plot_points()                     #plots contours