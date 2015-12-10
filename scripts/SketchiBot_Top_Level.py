#!/usr/bin/env python

#input: page read

#size tesseract text

#search image

#download image

#run contour filtering

#if image is less than certain # of waypoints, roll with it

#if not, choose next image



detector = EdgeDetector(image_path="../images/checkbox.jpeg") #creates edge detection class
detector.reconstruct_contours()     #makes contours
detector.sort_contours()            #sorts them to make the Neato's job easier
contours = detector.get_contours()  #actually gets image contours
size = detector.get_size()          #gets size of image
drawing = ContourFiltering(strokes = contours, imsize=size) #creates contour filtering class 
drawing.run_filter()                #runs filtering and centering methods on contours
strokes = drawing.get_strokes()             #returns strokes
#send waypoints to neato