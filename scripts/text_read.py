#!/usr/bin/env python
"""
Reads text from the Neato's camera using Tesseract

Requires Tesseract OCR and pytesseract
Requires Hunspell and pyhunspell
"""


import pytesseract
import hunspell
from cv_bridge import CvBridge
import PIL
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image

DECISION_THRESH = 3 # number of simultaneous readings to make a decision
speller = hunspell.HunSpell('/usr/share/hunspell/en_US.dic', '/usr/share/hunspell/en_US.aff')

class TextReader(object):
    """
    Text Reader Node
    """

    def __init__(self):
        rospy.init_node('text_reader')
        self.bridge = CvBridge  # used to convert ROS image messages to OpenCV
        self.image = None
        self.cv_image = None
        self.curr_reading = ''
        self.readings = ['' for i in range(DECISION_THRESH)]
        self.index = 0
        cv2.namedWindow('image')
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)

    def process_image(self, msg):
        """
        ROS Callback to store the viewed image
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB)
        self.image = PIL.Image.fromarray(cv_image)

    def run(self):
        """
        Debugging method to run indefinitely and print readings
        """
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
        """
        Method to run until we receive consecutive identical readings, and then spell-check it
        """

        r = rospy.Rate(2)
        decided = False
        streak_start = -1
        while not decided:
            if self.image != None:
                self.curr_reading = pytesseract.image_to_string(self.image)
                self.readings[self.index] = self.curr_reading.lower()

            if self.cv_image != None:
                cv2.imshow('image', self.cv_image)
                cv2.waitKey(5)

            if (self.readings[self.index] != ''
            and streak_start == self.index):
                # We have a valid reading and have looped around
                # to where we first read it, meaning we are done
                decided = True
                suggestion = []
                for word in self.readings[self.index].split():
                    # Correct spelling if the word isn't spelled correctly
                    if speller.spell(word):
                        suggestion.append(word)
                    else:
                        suggestion.append(speller.suggest(word)[0])
                suggestion = ' '.join(suggestion)
                return (self.readings[self.index], suggestion)

            elif (self.readings[self.index] == ''
            or self.readings[self.index] != self.readings[self.index-1]):
                # Our reading is blank or doesn't match the previous one, so we
                # restart the counter
                streak_start = -1

            elif (self.readings[self.index] != ''
            and self.readings[self.index] == self.readings[self.index-1]
            and streak_start == -1):
                # We have consecutive readings, start a recording a streak
                streak_start = self.index

            self.index = (self.index + 1) % DECISION_THRESH
            r.sleep()
