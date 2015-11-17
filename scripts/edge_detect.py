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
    img = cv2.imread(sys.argv[1],0)
    im2 = np.zeros(img.shape)
    edges = cv2.Canny(img, 100, 200)
    contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_TC89_KCOS)
    print len(contours)
    for contour in contours:
        cv2.circle(im2,(contour[0][0][0],contour[0][0][1]),3,(100,100,100))
        cv2.circle(im2,(contour[-1][0][0],contour[-1][0][1]),3,(100,100,100))
        for idx, point in enumerate(contour[0:-1]):
            p1 = (contour[idx][0][0], contour[idx][0][1])
            p2 = (contour[idx+1][0][0], contour[idx+1][0][1])
            cv2.arrowedLine(im2, p1, p2, (55,55,55))
        # cv2.drawContours(im2, [contour], -1, (55,55,55), 1)
            # cv2.imshow('contours', im2)
            # cv2.waitKey(0)
    cv2.imshow('edges', edges)
    cv2.waitKey(0)
