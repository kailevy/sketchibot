#!/usr/bin/env python
import rospy
from read_text import TextReader
from image_searcher import ImageSearch
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
