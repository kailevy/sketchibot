#!/usr/bin/env python

"""Adapted from http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html"""

import cv2
import numpy as np
import sys

edge_thresh = 1
max_low_thresh = 100
ratio = 3
kernel_size = 3

if __name__ == '__main__':
    img = cv2.imread(sys.argv[1],0);
    edges = cv2.Canny(img, 100, 200)
    cv2.imshow('edges', edges)
    cv2.waitKey(0)
