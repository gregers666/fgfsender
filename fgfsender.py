#!/usr/bin/python3
# -*- coding: utf8 -*-

#The FlightGear coordinates form a special body-fixed system, rotated from the standard body coordinate system about the y-axis by -180 degrees:
#
# The x-axis is positive toward the back of the vehicle.
# The y-axis is positive toward the right of the vehicle.
# The z-axis is positive upward, e.g., wheels typically have the lowest z values.

import socket

import time
import struct
import pymap3d
import binascii

from queue import Queue
from pynput import keyboard
import math
from Quaternion import Quat
from fgfslib import pos_msg


AC_LAT =  52.166
AC_LONG = 20.967
AC_HEIGHT = 120
X=0
Y=0
Z=0

queue = Queue()


CALLSIGN='SP-TWA'
NAME='Samolot'
MODEL='Aircraft/c172p/Models/c172p.xml'
CHAT='CHAT'

UDP_IP="192.168.1.16"
UDP_PORT=5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP

def go(ac_lat, ac_long, ac_height, x, y, z, callsign, model, udp_ip, udp_port):
    message_to_send = pos_msg(ac_lat, ac_long, ac_height, x, y, z, callsign, model)
    #print(message_to_send)
    sock.sendto(message_to_send, (udp_ip, udp_port))

    time.sleep(0.5)


while 1:
    go(AC_LAT, AC_LONG, AC_HEIGHT, X, Y, Z, CALLSIGN, MODEL, UDP_IP, UDP_PORT)
