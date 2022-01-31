#!/usr/bin/python3
# -*- coding: utf8 -*-
import socket

import time
import struct
import pymap3d
import binascii

from queue import Queue
from pynput import keyboard

global X
X=0
global Y
Y=0
global Z
Z=0

queue = Queue()

#TERMINATOR 
TT1=struct.pack('>b',0) # 0x00
TT2=struct.pack('>h',0) # 0x00 0x00


CALLSIGN='SP-TWAA'
NAME='Samolot'
MODEL='Aircraft/c172p/Models/c172p.xml'
CHAT='CHAT'

UDP_IP="192.168.1.29"
UDP_PORT=5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP

def pos_calc(pozycja_x, pozycja_y, pozycja_z):

    #4b
    # T_MsgHdr
    magic=0x46474653 #FGFS
    # > little endian
    message = struct.pack('>l', magic)

    #4b
    # The MP protocol version that is send with each packet (currently 1.1) uint32
    protoVer=0x00010001
    message += struct.pack('>l', protoVer)

    # T_MsgHdr::MsgId ::POS_DATA_ID or ::RESET_DATA_ID or CHAT_MSG_ID (not_used)
    # Message identifiers
    # deprecated message ids
    # 1 U1 = 1, // old CHAT_MSG_ID
    # 2 U2, // old pos data
    # 3 U3, // old pos data,
    # 4 U4, // old pos data,
    # 5 U5, // old pos data,
    # 6 U6, // RESET_DATA_ID
    # a "position" message, and the most trafficked
    # 7 POS_DATA,
    # 8 MP_2017_DATA_ID,
    # a ping packet is send verbatim back to sender
    # 9 PING,   // request
    # 10 PONG,   // answer

    #4b
    msgId=0x00000007
    message += struct.pack('>I', msgId)

    #4b
    # Absolute length of CHAT message bytes ID:2 bytes, CHAT_TEXT string 2 bytes
    # packet length
    msgLen=0x0000160 #e8 minimum  
    message += struct.pack('>I', msgLen)

    #4b
    # Visibility range
    requestedRangeNm=0x000007D0 #2000NM
    message += struct.pack('>I', requestedRangeNm)

    #4b
    # Reply Port - not used
    replyPort=0x00000000
    message += struct.pack('>I', replyPort)

    #8b
    # max callsign lenght 8
    callsign=bytes(CALLSIGN, encoding='raw_unicode_escape')
    message += callsign[:7]
    message += TT1# zero terminated


    #CHAT max 256b
    #chat=bytes(CHAT, encoding='raw_unicode_escape')
    #message += chat[:256]
    #message += TT2 # zero terminated


    #96b
    #Model  max length = 96 ie. "/model/foo/aero.xml" Name of the aircraft model.
    model=bytes(MODEL, encoding='raw_unicode_escape')
    message += model[:96]
    for i in range(1, 96 -len(model)):
        message += b'\x00'
    message += TT1 # zero terminated

    #8b
    #time   epoch time Time when this packet was generated.
    NOW=time.time()
    print('Time %s' % NOW)
    now=struct.pack('>d',  NOW)
    message += now

    #8b
    #lag
    lag=0x02
    message += struct.pack('>d', lag)

    #24b
    #PositionMsg https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
    POS=pymap3d.aer2ecef(0,0,0,52.166,20.967, 120) 

    # i.e. 6378137.0 -> b'AXT\xa6@\x00\x00\x00' # # # -> binascii.hexlify daje b'415854a640000000' i tak powinno być w odczycie z pakietu

    print('x=%s, y=%s, z= %s' % ( POS[0], POS[1], POS[2]))
    POS_X = struct.pack('>d', POS[0])
    POS_Y = struct.pack('>d', POS[1]) 
    POS_Z = struct.pack('>d', POS[2]) 
    POS_X1=POS_X
    POS_Y1=POS_Y
    POS_Z1=POS_Z 


    #print('POS_X: %s' % POS_X)
    #print('POS_Y: %s' % POS_Y)
    #print('POS_Z %s' % POS_Z)
    message += POS_X
    message += POS_Y
    message += POS_Z

    # 12b
    #hdgorientation float[3] Orientation wrt the earth centered frame, stored in the angle axis representation where the angle is coded into the axis length.
    ori_x=pozycja_x
    ori_y=pozycja_y
    ori_z=pozycja_z
    message += struct.pack ('>f', ori_x)
    message += struct.pack ('>f', ori_y)
    message += struct.pack ('>f', ori_z)


    # 12b
    #linearVel float[3] Linear velocity wrt the earth centered frame measured in the earth centered frame.
    linearVel_X=0
    linearVel_Y=0
    linearVel_Z=0
    message += struct.pack('>I', linearVel_X)
    message += struct.pack('>I', linearVel_Y)
    message += struct.pack('>I', linearVel_Z)


    # 12b
    #angularVel float[3] Angular velocity wrt the earth centered frame measured in the earth centered frame.
    angularVel_X=0
    angularVel_Y=0
    angularVel_Z=0
    message += struct.pack('>I', angularVel_X)
    message += struct.pack('>I', angularVel_Y)
    message += struct.pack('>I', angularVel_Z)


    # 12b
    #linearAccel float[3] Linear acceleration wrt the earth centered frame measured in the earth centered frame.
    linearAccel_X=0
    linearAccel_Y=0
    linearAccel_Z=0
    message += struct.pack('>I', linearAccel_X)
    message += struct.pack('>I', linearAccel_Y)
    message += struct.pack('>I', linearAccel_Z)

    # 12b
    #angularAccel float[3] Angular acceleration wrt the earth centered frame measured in the earth centered frame.
    angularAccel_X=0
    angularAccel_Y=0
    angularAccel_Z=0
    message += struct.pack('>I', angularAccel_X)
    message += struct.pack('>I', angularAccel_Y)
    message += struct.pack('>I', angularAccel_Z)

    ####### FGExternal motion data
    #  simulation time when this packet was generated
    #  double time;
    NOW=time.time()
    now_sim=struct.pack('>d',  NOW)
    message += now_sim

    #  the artificial lag the client should stay behind the average
    #  simulation time to arrival time difference
    lag1=0x03 #3FB999999999999a
    message += struct.pack('>q', lag1)

    print('POS_X1: %s' % POS_X1)
    print('POS_Y1: %s' % POS_Y1)
    print('POS_Z1: %s' % POS_Z1)

    message += POS_X1
    message += POS_Y1
    message += POS_Z1


    #  position wrt the earth centered frame
    #SGVec3d position;
    # T_PositionMsg https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
    #position   double[3] Position wrt the earth centered frame.
    message += struct.pack ('>Q', 0xc154c68286FFFFFF)
    message += struct.pack('>Q', 0xc143531d84FFFFFF)
    message += struct.pack('>Q', 0x3ffacaa333FFFFFF)

    #  orientation wrt the earth centered frame
    #SGQuatf orientation;
    #hdgorientation float[3] Orientation wrt the earth centered frame, stored in the angle axis representation where the angle is coded into the axis length.
    ori_x=0
    ori_y=0
    ori_z=0
    message += struct.pack ('>f', ori_x)
    message += struct.pack('>f', ori_y)
    message += struct.pack('>f', ori_z)

    #  linear velocity wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f linearVel;

    #linearVel float[3] Linear velocity wrt the earth centered frame measured in the earth centered frame.
    linearVel_X=0
    linearVel_Y=0
    linearVel_Z=0
    message += struct.pack('>f', linearVel_X)
    message += struct.pack('>f', linearVel_Y)
    message += struct.pack('>f', linearVel_Z)


    #  angular velocity wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f angularVel;
    #angularVel float[3] Angular velocity wrt the earth centered frame measured in the earth centered frame.
    angularVel_X=0
    angularVel_Y=0
    angularVel_Z=0
    message += struct.pack('>f', angularVel_X)
    message += struct.pack('>f', angularVel_Y)
    message += struct.pack('>f', angularVel_Z)


    #  linear acceleration wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f linearAccel;
    #linearAccel float[3] Linear acceleration wrt the earth centered frame measured in the earth centered frame.
    linearAccel_X=0
    linearAccel_Y=0
    linearAccel_Z=0
    message += struct.pack('>f', linearAccel_X)
    message += struct.pack('>f', linearAccel_Y)
    message += struct.pack('>f', linearAccel_Z)


    #  angular acceleration wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f angularAccel;
    #angularAccel float[3] Angular acceleration wrt the earth centered frame measured in the earth centered frame.
    angularAccel_X=0
    angularAccel_Y=0
    angularAccel_Z=0
    message += struct.pack('>f', angularAccel_X)
    message += struct.pack('>f', angularAccel_Y)
    message += struct.pack('>f', angularAccel_Z)


    return message

def on_press(key):
    global X
    global Y
    global Z
    
    try:
        if key == keyboard.Key.right:
            X += 0.1
        if key == keyboard.Key.left:
            X -= 0.1
        if key == keyboard.Key.up:
            Y += 0.1
        if key == keyboard.Key.down:
            Y -= 0.1
        if key == keyboard.Key.page_up:
            Z += 0.1
        if key == keyboard.Key.page_down:
            Z -= 0.1

    except AttributeError:
        pass#   print('other')


# ~ def on_release(key):
     # ~ if key == keyboard.Key.esc:
        # ~ # Stop listener
        # ~ return False


# ...or, in a non-blocking fashion:
listener = keyboard.Listener(on_press=on_press) #    #on_release=on_release)
listener.start()


def go():
    global X, Y, Z
   
    message_to_send = pos_calc(X,Y,Z)
    #print(hex(len(message)))
    #print(message_to_send)
    sock.sendto(message_to_send, (UDP_IP, UDP_PORT))
    print(X,Y,Z)
    time.sleep(0.2)
    
X=0
while 1:
    go()
