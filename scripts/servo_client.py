#!/usr/bin/env python

"""
A simple echo client

This code must go into neato_node bringup_minimal.launch:

    <node name="marker_node" pkg="sketchibot" type="servo_client.py" output="screen">
        <param name="host" value="$(arg host)" />
    </node>
"""

import rospy
import socket
from std_msgs.msg import String

class ServoClient(object):

    def __init__(self):
        rospy.init_node('servo_server')
        self.sub = rospy.Subscriber('servo_command', String, self.process_command)
        self.host = rospy.get_param('~host')
        self.port = 50001
        self.size = 1024
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.host,self.port))

    def process_command(self,msg):
        self.s.send(msg.data)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()
        self.s.close()

if __name__ == '__main__':
    sketchibot = ServoClient()
    sketchibot.run()
