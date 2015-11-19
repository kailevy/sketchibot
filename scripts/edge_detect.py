#!/usr/bin/env python

"""Adapted from http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html"""

import cv2
import numpy as np
import sys

class EdgeDetector(object):
    def __init__(self, image_path, canny_param1 = 100, canny_param2 = 200):
        self.img = cv2.imread(image_path, 0)
        self.edges = cv2.Canny(self.img, canny_param1, canny_param2)
        self.contours, self.hierarchy = cv2.findContours(self.edges, cv2.RETR_TREE,
            cv2.CHAIN_APPROX_TC89_KCOS) #perhaps change this parameters?

    def display_image(self):
        cv2.imshow('image', self.img)
        cv2.waitKey(0)

    def display_edges(self):
        cv2.imshow('edges', self.edges)
        cv2.waitKey(0)

    def animate_contours(self):
        im2 = np.zeros(self.img.shape)
        for contour in self.contours:
            cv2.circle(im2,(contour[0][0][0],contour[0][0][1]),1,(255,255,255))
            cv2.circle(im2,(contour[-1][0][0],contour[-1][0][1]),1,(255,255,255))
            for idx, point in enumerate(contour[0:-1]):
                p1 = (contour[idx][0][0], contour[idx][0][1])
                p2 = (contour[idx+1][0][0], contour[idx+1][0][1])
                cv2.arrowedLine(im2, p1, p2, (255,255,255))
                cv2.imshow('contours', im2)
                cv2.waitKey(1)
        cv2.waitKey(0)

    def sort_contours(self):
        # TODO: Implement this, sort them for least amount of travelling between strokes
        pass

    def get_contours(self):
        return self.contours


if __name__ == '__main__':
    image = sys.argv[1]
    det = EdgeDetector(image)
    det.display_image()
    det.display_edges()
    det.animate_contours()
    print len(det.get_contours())
