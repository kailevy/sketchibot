#!/usr/bin/env python
import rospy
from text_read import TextReader
from image_search import ImageSearcher
from edge_detect import EdgeDetector
from contour_filtering import ContourFiltering
from localization import Sketchibot

waypt_limit = 500.0

#input: page read

#size tesseract text

#search image

has_found = False
skip_counter = 0
searcher = ImageSearcher()
while not has_found:
	images = searcher.find_image(query=query, skip=skip_counter, filters=[MEDIUM, DRAWING])
	for picture in images:
		detector = EdgeDetector(image=picture) #creates edge detection class
		detector.reconstruct_contours()     #makes contours
		detector.sort_contours()            #sorts them to make the Neato's job easier
		contours = detector.get_contours()  #actually gets image contours
		size = detector.get_size()          #gets size of image
		drawing = ContourFiltering(strokes = contours, imsize=size) #creates contour filtering class 
		drawing.run_filter()                #runs filtering and centering methods on contours
		waypts = drawing.get_number_of_waypoints()  #gets number of waypoints
		if wapts < waypt_limit:
			strokes = drawing.get_strokes()             #returns strokes
			has_found = True
			break
	skip_counter +=10
			


#download image

#run contour filtering

#if image is less than certain # of waypoints, roll with it

#if not, choose next image







#send waypoints to neato
