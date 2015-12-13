"""
Canny edge detection, contour finding, and sorting
Adapted from http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
"""

import cv2
import numpy as np
import sys
from image_searcher import ImageSearch
from scipy.linalg import norm
from scipy import sum, average

class EdgeDetector(object):
    """
    Runs Canny edge detection on an image and converts it to a series of contours
    """
    def __init__(self, image_path=None, image=None):
        if image != None:
            self.img = image
        else:
            self.img = cv2.imread(image_path, 0)
        high_thresh, thresh_im = cv2.threshold(self.img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        low_thresh = 0.5*high_thresh
        edges = cv2.Canny(self.img, low_thresh, high_thresh)
        self.contours, self.hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS) #perhaps change this parameters?
        self.edges = cv2.Canny(self.img, low_thresh, high_thresh)

    def reconstruct_contours(self):
        """
        Method to remove unnecessary list-nesting in the contours,
        as well as filter out 'holes' (which are counter-clockwise contours)
        """
        temp_contours = []
        for contour in self.contours:
            if cv2.contourArea(contour, True) < 0:
                # Area less than 0 means it is an object, not a hole
                temp_contours.append([vector[0].astype('float') for vector in contour])
        self.contours = temp_contours

    def display_image(self):
        """
        Displays the image
        """
        cv2.imshow('image', self.img)
        cv2.waitKey(0)

    def display_edges(self):
        """
        Displays the edges
        """
        cv2.imshow('edges', self.edges)
        cv2.waitKey(0)

    def animate_contours(self, path='default'):
        """
        Animates the found contours in order
        """
        self.im2 = np.zeros(self.img.shape)
        if path == 'default':
            path = self.contours
        elif path == 'sorted':
            path = self.path
        for contour in path:
            for idx, point in enumerate(contour[0:-1]):
                p1 = (int(contour[idx][0]), int(contour[idx][1]))
                p2 = (int(contour[idx+1][0]), int(contour[idx+1][1]))
                cv2.arrowedLine(self.im2, p1, p2, (255,255,255))
                cv2.imshow('contours', self.im2)
                cv2.waitKey(1)
        cv2.waitKey(0)


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

def compare_images(img1, img2):
    """
    Helper function from:
    http://stackoverflow.com/questions/189943/how-can-i-quantify-difference-between-two-images
    """
    # normalize to compensate for exposure difference, this may be unnecessary
    # consider disabling it
    # calculate the difference and its norms
    diff = (img1) - (img2)  # elementwise for scipy arrays
    z_norm = norm(diff.ravel(), 1)  # one norm
    return z_norm

def normalize(arr):
    """
    normalize the image array by taking (each element - mean) / standard dev
    """
    arr_mean = arr.mean()
    std_dev = arr.std()
    return (arr - arr_mean) / std_dev

if __name__ == '__main__':
    image = sys.argv[1]
    det = EdgeDetector(image_path=image)
    det.reconstruct_contours()
    det.sort_contours()
    det.display_image()
    det.display_edges()
    det.animate_contours('sorted')
