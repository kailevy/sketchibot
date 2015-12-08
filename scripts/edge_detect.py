#!/usr/bin/env python

"""Adapted from http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html"""

import cv2
import numpy as np
import sys
from image_searcher import ImageSearch

class EdgeDetector(object):
    def __init__(self, image_path=None, image=None):
        if image != None:
            self.img = image
        else:
            self.img = cv2.imread(image_path, 0)
        high_thresh, thresh_im = cv2.threshold(self.img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        low_thresh = 0.5*high_thresh
        self.edges = cv2.Canny(self.img, low_thresh, high_thresh)
        self.contours, self.hierarchy = cv2.findContours(self.edges, cv2.RETR_TREE,
            cv2.CHAIN_APPROX_TC89_KCOS) #perhaps change this parameters?

    def reconstruct_contours(self):
        """
        Method to remove unnecessary nesting in the contours
        """
        temp_contours = []
        for contour in self.contours:
            temp_contours.append([vector[0].astype('float') for vector in contour])
        self.contours = temp_contours

    def display_image(self):
        cv2.imshow('image', self.img)
        cv2.waitKey(0)

    def display_edges(self):
        cv2.imshow('edges', self.edges)
        cv2.waitKey(0)

    def animate_contours(self, path='default'):
        im2 = np.zeros(self.img.shape)
        if path == 'default':
            path = self.contours
        elif path == 'sorted':
            path = self.path
        for contour in path:
            cv2.circle(im2,(int(contour[0][0]),int(contour[0][1])),1,(255,255,255))
            cv2.circle(im2,(int(contour[-1][0]),int(contour[-1][1])),1,(255,255,255))
            for idx, point in enumerate(contour[0:-1]):
                p1 = (int(contour[idx][0]), int(contour[idx][1]))
                p2 = (int(contour[idx+1][0]), int(contour[idx+1][1]))
                cv2.arrowedLine(im2, p1, p2, (255,255,255))
                cv2.imshow('contours', im2)
                cv2.waitKey(1)
        cv2.waitKey(0)

    def make_distance_graph(self):
        # Obsolete?
        self.distances = {}
        for index, l in enumerate(self.contours):
            tmp = []
            end = l[-1]
            for index2, l2 in enumerate(self.contours):
                if index != index2:
                    tmp.append([index2, np.linalg.norm(end-l2[0])])
            self.distances[index] = tmp

    def sort_contours(self, start_point=(0,0)):
        """
        Calculates path through all strokes by using min distance approach to TSP (not most efficient)
        Goes to the closest stroke from each one
        """
        unvisited = {}
        for index, contour in enumerate(self.contours):
            unvisited[index] = contour
        start = min(unvisited, key=lambda x: np.linalg.norm(start_point-unvisited[x][0]))
        path = [unvisited[start]]
        unvisited.pop(start, None)
        while unvisited:
            end = path[-1][-1]
            nearest = min(unvisited, key=lambda x: np.linalg.norm(end-unvisited[x][0]))
            path.append(unvisited[nearest])
            unvisited.pop(nearest, None)
        self.path = path

    def get_contours(self):
        return self.contours

    def get_size(self):
        return self.img.shape

if __name__ == '__main__':
    image = sys.argv[1]
    # searcher = ImageSearch()
    # image = searcher.find_image('cow')[0]
    det = EdgeDetector(image_path=image)
    det.reconstruct_contours()
    # det.display_image()
    det.display_edges()
    det.sort_contours()
    # det.animate_contours()
    det.animate_contours('sorted')
    strokes = det.get_contours()
    # print strokes
    size = det.get_size()
