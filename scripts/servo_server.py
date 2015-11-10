#!/usr/bin/env python

"""
A simple echo server
"""

import socket
from servo import ServoControl

host = ''
port = 50001
backlog = 5
size = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host,port))
s.listen(backlog)
marker = ServoControl(11)
while 1:
    client, address = s.accept()
    while True:
        data = client.recv(size)
        if data:
            marker.move_marker(data)
        else:
            break
    client.close()
