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
UDP_IP = "172.17.0.2"
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

from queue import Queue
import math
from Quaternion import Quat
from fgfslib import pos_msg
from walrus import *

db = Walrus(host = REDIS_IP, port = REDIS_PORT, db = REDIS_DB)
queue = Queue()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP


def go(ac_lat, ac_long, ac_height, x, y, z, callsign, model, udp_ip, udp_port):
    message_to_send = pos_msg(ac_lat, ac_long, ac_height, x, y, z, callsign, model)
    #print(message_to_send)
    sock.sendto(message_to_send, (udp_ip, udp_port))
    time.sleep(0.1)


test = {}
airport = {}

test[0] = {"ac_lat":AC_LAT, "ac_long":AC_LONG, "ac_height":AC_HEIGHT, "x":X, "y":Y, "z":Z, "callsign":CALLSIGN, "model":MODEL }
test[1] = {"ac_lat":AC_LAT+0.002, "ac_long":AC_LONG, "ac_height":AC_HEIGHT, "x":X, "y":Y, "z":Z, "callsign":"HANS", "model":MODEL }
test[2] = {"ac_lat":AC_LAT+0.001, "ac_long":AC_LONG, "ac_height":AC_HEIGHT, "x":X, "y":Y, "z":Z, "callsign":"KLOSS", "model":MODEL }


#airport[0] = db.Hash(0)
#airport[1] = db.Hash(1)

airport[0] = test[0]
airport[1] = test[1]
airport[2] = test[2]


print("Start")
while 1:
    for ac in airport.items():
#        print(ac[1])
        go( ac[1]["ac_lat"], ac[1]["ac_long"], ac[1]["ac_height"], ac[1]["x"], ac[1]["y"], ac[1]["z"], ac[1]["callsign"], ac[1]["model"], UDP_IP, UDP_PORT )
