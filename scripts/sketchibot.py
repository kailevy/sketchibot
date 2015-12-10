#!/usr/bin/env python
import rospy
from text_read import TextReader
from image_searcher import ImageSearcher
from edge_detect import EdgeDetector
from contour_filtering import ContourFiltering
from localization import Sketchibot


#input: page read

#size tesseract text

#search image

#download image

#run contour filtering

#if image is less than certain # of waypoints, roll with it

#if not, choose next image

#send waypoints to neato
