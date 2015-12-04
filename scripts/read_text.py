#!/usr/bin/env python

import pytesseract
import rospy
from cv_bridge import CvBridge
import PIL
from sensor_msgs.msg import Image
import cv2
import numpy as np

class TextReader(object):

    def __init__(self):
        rospy.init_node('text_reader')
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.image = None
        self.cv_image = None
        self.reading = ''
        cv2.namedWindow('image')
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)

    def process_image(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB)
        self.image = PIL.Image.fromarray(cv_image)
        # if self.reading == '':
        self.reading = pytesseract.image_to_string(self.image)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print self.reading
            if self.cv_image != None:
                cv2.imshow('image', self.cv_image)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    text_reader = TextReader()
    text_reader.run()
