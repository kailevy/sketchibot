#!/usr/bin/env python
"""
Requires Tesseract ORC and pytesseract
Requires Hunspell and pyhunspell
"""


import pytesseract
import hunspell
import rospy
from cv_bridge import CvBridge
import PIL
from sensor_msgs.msg import Image
import cv2
import numpy as np

DECISION_THRESH = 3 # number of simultaneous readings to make a decision
speller = hunspell.HunSpell('/usr/share/hunspell/en_US.dic', '/usr/share/hunspell/en_US.aff')

class TextReader(object):

    def __init__(self):
        rospy.init_node('text_reader')
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.image = None
        self.cv_image = None
        self.curr_reading = ''
        self.readings = ['' for i in range(DECISION_THRESH)]
        self.index = 0
        cv2.namedWindow('image')
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)

    def process_image(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB)
        self.image = PIL.Image.fromarray(cv_image)


    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.image != None:
                self.curr_reading = pytesseract.image_to_string(self.image)
                self.readings[self.index] = self.curr_reading
                self.index = (self.index + 1) % DECISION_THRESH
            print self.curr_reading
            if self.cv_image != None:
                cv2.imshow('image', self.cv_image)
                cv2.waitKey(5)
            r.sleep()

    def get_reading(self):
        r = rospy.Rate(2)
        decided = False
        streak_start = -1
        while not decided:
            if self.image != None:
                self.curr_reading = pytesseract.image_to_string(self.image)
                self.readings[self.index] = self.curr_reading.lower() # maybe have strip here?

            if self.cv_image != None:
                cv2.imshow('image', self.cv_image)
                cv2.waitKey(5)

            # print self.readings, self.index, streak_start
            # print streak_start == self.index
            if (self.readings[self.index] != ''
            and streak_start == self.index):
                decided = True
                return (self.readings[self.index], speller.suggest(self.readings[self.index])[0])
            elif (self.readings[self.index] == ''
            or self.readings[self.index] != self.readings[self.index-1]):
                streak_start = -1
            elif (self.readings[self.index] != ''
            and self.readings[self.index] == self.readings[self.index-1]
            and streak_start == -1):
                streak_start = self.index

            self.index = (self.index + 1) % DECISION_THRESH
            r.sleep()



if __name__ == '__main__':
    text_reader = TextReader()
    print text_reader.get_reading()
