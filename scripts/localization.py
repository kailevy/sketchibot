#!/usr/bin/env python

import rospy
import tf
import numpy as np
import math
import time
import rospkg
import cv2

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String

from edge_detect import EdgeDetector
from contour_filtering import ContourFiltering

VEL_ANGULAR_LIM = 0.2
VEL_LINEAR_LIM = 0.2

""" Converts a translation and rotation component to a Pose object """
def convert_to_pose(translation, rotation):
    return Pose(position=Point(x=translation[0],
                               y=translation[1],
                               z=translation[2]),
                orientation=Quaternion(x=rotation[0],
                                       y=rotation[1],
                                       z=rotation[2],
                                       w=rotation[3]))

""" Extracts position and angle information from a Pose object """
def convert_pose_to_xy_and_theta(pose):
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

""" Main node that commands the Neato to go to waypoints, given a series of points from an image """
class Sketchibot(object):

    """ Initialzies important variables and nodes """
    def __init__(self, strokes):
        rospy.init_node('localization')
 
        # Publishes directly to the navigation stack
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_marker = rospy.Publisher('/servo_command', String, queue_size=10)

        # Gets current position of the Neato
        rospy.Subscriber('/position', PoseStamped, self.position_callback)

        self.page_size = (1.5,1.5)

        # Current and final state variables of the Neato
        self.x,   self.y,   self.th   = (0, 0, 0)
        self.x_f, self.y_f, self.th_f = (0, 0, 0)

        # Previous and current waypoint visited
        self.prev_wp = (0, 0)
        self.curr_wp = (0, 0)

        # Draws all of the paths taken onto an image
        im_x, im_y = 512 * self.page_size[0], 512 * self.page_size[1]
        self.path_image = np.zeros((im_x, im_y, 3), np.uint8)
        cv2.imshow("Path", self.path_image)
        cv2.waitKey(50)

        self.contours = strokes

    """ Callback function for Neato's current position """
    def position_callback(self, msg):
        self.x, self.y, self.th = convert_pose_to_xy_and_theta(msg.pose)
        print self.x, self.y, self.th

    """ Publishes a waypoint to the Neato, with the map as the coordinate frame
            pos - delta in translational motion
            rot - delta in heading """
    def push_waypoint(self, pos, rot):
        x, y = pos  # Location of the next goal

        self.x_f, self.y_f = pos
        self.th_f = rot

        # Rotates the Neato towards the next goal
        self.rotate_neato()

        # Drives the Neato forwards until goal is reached
        self.forwards_neato()

    """ Checks if goal has been reached """
    def reached_goal(self):
        dist = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2)
        return dist < 0.03

    """ Checks if the Neato has rotated towards the target goal """
    def correct_heading(self):
        return abs(self.th_f - self.th) < 0.1 or abs(abs(self.th_f - self.th) - 2*np.pi) < 0.1

    """ Moves the Neato until it is at the waypoint """
    def forwards_neato(self):
        while not self.reached_goal():
            if not self.correct_heading():
                self.rotate_neato()
            # print self.initial_x, self.initial_y, self.x, self.y, self.x_f, self.y_f, self.th, self.th_f
            self.th_f = self.calc_theta(self.x, self.y, self.x_f, self.y_f)

            xy_error = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2)
            th_error = self.calc_th_error()

            K_xy = 0.3  # Proportional control
            K_th = 0.1

            vel = Twist()
            vel.linear.x = min(K_xy * xy_error + 0.05, VEL_LINEAR_LIM)
            omega = K_th * th_error #+ 0.1 * np.sign(th_error)
            if omega < -VEL_ANGULAR_LIM*2:
                omega = -VEL_ANGULAR_LIM*2
            elif omega > VEL_ANGULAR_LIM*2:
                omega = VEL_ANGULAR_LIM*2
            vel.angular.z = omega

            self.pub_vel.publish(vel)
        self.curr_wp = (self.x, self.y)
        self.draw_visited()
        self.prev_wp = self.curr_wp

    """ Rotates the Neato until its heading is correct """
    def rotate_neato(self):
        while not self.correct_heading():
            dist = math.sqrt((self.x_f - self.x)**2 + (self.y_f - self.y)**2)
            if dist < 0.075:
                return

            th_error = self.calc_th_error()
            K = 0.3  # Proportional control

            vel = Twist()
            omega = K * th_error + 0.1 * np.sign(th_error)
            if omega < -VEL_ANGULAR_LIM:
                omega = -VEL_ANGULAR_LIM
            elif omega > VEL_ANGULAR_LIM:
                omega = VEL_ANGULAR_LIM
            vel.angular.z = omega
            self.pub_vel.publish(vel)

    """ Calculates the th_error, accounts for wrapping effect """
    def calc_th_error(self):
            th_error1 = self.th_f - self.th
            th_error2 = self.th_f - self.th - 2*np.pi
            th_error3 = self.th_f - self.th + 2*np.pi
            mag1, mag2, mag3 = abs(th_error1), abs(th_error2), abs(th_error3)
            min_mag = min(mag1, mag2, mag3)

            if min_mag == mag1:
                return th_error1
            elif min_mag == mag2 or min_mag == mag3:
                return -th_error1


    """ Calculates angle between two points """
    def calc_theta(self, x1, y1, x2, y2):
        return math.atan2(y2-y1, x2-x1)

    # """ Gets a list of points to follow """
    # def get_contours(self):
    #     #edge detection stuff
    #     rospack = rospkg.RosPack()
    #     path = rospack.get_path('sketchibot')
    #     detector = EdgeDetector(image_path=path+"/images/cow.png") #creates edge detection class
    #     detector.reconstruct_contours()     #makes contours
    #     detector.sort_contours()            #sorts them to make the Neato's job easier
    #     contours = detector.get_contours()  #actually gets image contours
    #     size = detector.get_size()          #gets size of image
    #     drawing = ContourFiltering(strokes = contours,imsize=size,pagesize=self.page_size) #creates contour filtering class 
    #     drawing.run_filter()                #runs filtering and centering methods on contours
    #     waypts = drawing.get_number_of_waypoints()  #gets number of waypoints
    #     strokes = drawing.get_strokes()             #returns strokes
    #     return strokes

    """ Draws all of the strokes onto an OpenCV window """
    def draw_visited(self):
        pt1_x = int(self.prev_wp[0] * self.page_size[0])
        pt1_y = int(self.prev_wp[1] * self.page_size[1])
        pt2_x = int(self.curr_wp[0] * self.page_size[0])
        pt2_y = int(self.curr_wp[1] * self.page_size[1])

        cv2.line(self.path_image, (pt1_x, pt1_y), (pt2_x, pt2_y), (0,0,255), 1)
        cv2.imshow("Path", self.path_image)
        cv2.waitKey(10)

    """ Main loop that sends velocity commands to the Neato """
    def run(self):
        # Waits until everything has been initialized
        rospy.wait_for_message('/position', PoseStamped, timeout=5)

        first = True
        prev = [0, 0]

        r = rospy.Rate(10)
        if not rospy.is_shutdown():
            for i in self.contours:
                for j in i:
                    if first == True:
                        first = False
                        self.pub_marker.publish(String("0"))
                        print "pen up"
                    else:
                        self.pub_marker.publish(String("1"))
                        print "pen down"
                    pos = [j[0], -j[1]]
                    rot = self.calc_theta(prev[0], prev[1], pos[0], pos[1])

                    self.push_waypoint(pos, rot)
                    prev = pos
                print "contour"
                first = True # Marker should be up when going to the first point of each new contour

        self.pub_vel.publish(Twist())
        cv2.waitKey(0)

if __name__ == '__main__':
        #edge detection stuff
    # rospack = rospkg.RosPack()
    # path = rospack.get_path('sketchibot')
    # detector = EdgeDetector(image_path=path+"/images/cow.png") #creates edge detection class
    # detector.reconstruct_contours()     #makes contours
    # detector.sort_contours()            #sorts them to make the Neato's job easier
    # contours = detector.get_contours()  #actually gets image contours
    # size = detector.get_size()          #gets size of image
    # drawing = ContourFiltering(strokes = contours,imsize=size,pagesize=(1.5,1.5)) #creates contour filtering class 
    # drawing.run_filter()                #runs filtering and centering methods on contours
    # waypts = drawing.get_number_of_waypoints()  #gets number of waypoints
    # strokes = drawing.get_strokes()             #returns strokes
    # print strokes

    strokes = np.array([[[0,1], [1,1], [1,0], [0,0]]])
    node = Sketchibot(strokes)
    node.run()