#!/usr/bin/python3
# -*- coding: utf8 -*-

#The FlightGear coordinates form a special body-fixed system, rotated from the standard body coordinate system about the y-axis by -180 degrees:
#
# The x-axis is positive toward the back of the vehicle.
# The y-axis is positive toward the right of the vehicle.
# The z-axis is positive upward, e.g., wheels typically have the lowest z values.

CALLSIGN = 'SP-TWA'
NAME = 'Samolot'
#MODEL = 'Aircraft/c172p/Models/c172p.xml'
MODEL = '/media/gregers/GRY/FGFS/fgdata/Aircraft/737-200/Models/737-200.xml'

CHAT = 'CHAT'
REDIS_IP = "127.0.0.1"
REDIS_PORT = 6379
REDIS_DB = 0
UDP_IP = "192.168.1.10"
UDP_PORT = 5000

AC_LAT =  52.166
AC_LONG = 20.967
AC_HEIGHT = 120
X = 0
Y = 0
Z = 0

import socket

import time
import struct
import pymap3d
import binascii

#from queue import Queue
import math
from Quaternion import Quat
from fgfslib import pos_msg
from redisworks import Root




root = Root(host='localhost', port=6379, db=0)

#queue = Queue()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP


def go(ac_lat, ac_long, ac_height, x, y, z, callsign, model, udp_ip, udp_port):
    message_to_send = pos_msg(ac_lat, ac_long, ac_height, x, y, z, callsign, model)
    #print(message_to_send)
    sock.sendto(message_to_send, (udp_ip, udp_port))
    time.sleep(0.1)


test = {}
root.flush()
root.airport[0] = {0:0} # strange init redisworks

for x in range(0, 30):
    test[x] = {"ac_lat":AC_LAT-x/1000, "ac_long":AC_LONG, "ac_height":AC_HEIGHT+x*10, "x":X, "y":Y, "z":Z, "callsign":str(x), "model":MODEL }
    root.airport[x] = test[x]


#root.airport[0] = test[0]
#root.airport[1] = test[1]
#root.airport[2] = test[2]


print("Start")
while 1:
    try:
        for ac in root.airport:
            go( ac["ac_lat"], ac["ac_long"], ac["ac_height"], ac["x"], ac["y"], ac["z"], ac["callsign"], ac["model"], UDP_IP, UDP_PORT )
    except KeyError as k:
        pass
