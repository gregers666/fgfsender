#!/usr/bin/python3
# -*- coding: utf8 -*-

import time
import struct
import pymap3d
import binascii
import math
from Quaternion import Quat


# The x-axis is positive toward the back of the vehicle.
# The y-axis is positive toward the right of the vehicle.
# The z-axis is positive upward, e.g., wheels typically have the lowest z values.

#def __init__(aircraft_lat, aircraft_long, aircraft_height, angle_x, angle_y, angle_z, callsign, model):
# TERMINATOR 
__TT1 = struct.pack('>b',0) # 0x00
__TT2 = struct.pack('>h',0) # 0x00 0x00

def _llh2ecef(lat, lon, alt):
    #WGS84 reference ellipsoid constants
    wgs84_a = 6378137.0
    wgs84_b = 6356752.314245
    wgs84_e2 = 0.0066943799901975848
    #wgs84_a2 = wgs84_a**2 #to speed things up a bit
    #wgs84_b2 = wgs84_b**2
    
    lat *= (math.pi / 180.0)
    lon *= (math.pi / 180.0)
    
    n = lambda x: wgs84_a / math.sqrt(1 - wgs84_e2*(math.sin(x)**2))
    
    x = (n(lat) + alt)*math.cos(lat)*math.cos(lon)
    y = (n(lat) + alt)*math.cos(lat)*math.sin(lon)
    z = (n(lat)*(1-wgs84_e2)+alt)*math.sin(lat)
    
    return [x,y,z]

def _ac_rotate (lat, lon, alt, angle_x, angle_y, angle_z, vel, vs, turnrate):
    #position, orientation, linear vel, angular vel, linear accel, angular accel (accels unused), 0
    #position is in ECEF format -- same as mlat uses. what luck!
    pos = _llh2ecef(lat, lon, alt * 0.3048) #alt is in meters!

    #get the rotation quaternion to rotate to local reference frame from lat/lon
    rotquat = Quat([lat, lon])
    #get the quaternion corresponding to aircraft orientation
    acquat = Quat([angle_x, angle_y, angle_z])
    #rotate aircraft into ECEF frame
    ecefquat = rotquat * acquat
    #get it in angle/axis representation
    (angle, axis) = ecefquat._get_angle_axis()
    orientation = angle * axis
    
    kts_to_ms = 0.514444444 #convert kts to m/s
    vel_ms = vel * kts_to_ms
    velvec = (vel_ms,0,0) #velocity vector in m/s -- is this in the local frame? looks like [0] is fwd vel,
                               #we'll pretend the a/c is always moving the dir it's pointing
    turnvec = (0,0,turnrate * (math.pi / 180.) ) #turn rates in rad/s [roll, pitch, yaw]
    #accelvec = (0,0,0)
    #turnaccelvec = (0,0,0)
    #posfmt = '!96s' + 'd' + 'd' + '3d' + '3f' + '3f' + '3f' + '3f' + '3f' + 'I'
    return orientation[0], orientation[1], orientation[2] #pos[0], pos[1], pos[2], 
            

def pos_msg(aircraft_lat, aircraft_long, aircraft_height, angle_x, angle_y, angle_z, callsign, model):
    """ pos_msg returns FlightGear multiplayer message with 
    aircraft position (lat, long, height, angles x,y,z 
    in reference to north, callsign, flightgear aircraft model"""

    #4b
    # T_MsgHdr
    magic = 0x46474653 #FGFS
    # > little endian
    message = struct.pack('>l', magic)

    #4b
    # The MP protocol version that is send with each packet (currently 1.1) uint32
    protoVer = 0x00010001
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
    msgId = 0x00000007
    message += struct.pack('>I', msgId)

    #4b
    # Absolute length of CHAT message bytes ID:2 bytes, CHAT_TEXT string 2 bytes
    # packet length
    msgLen = 0x0000148 #e8 minimum  
    message += struct.pack('>I', msgLen)

    #4b
    # Visibility range
    requestedRangeNm = 0x000007D0 #2000NM
    message += struct.pack('>I', requestedRangeNm)

    #4b
    # Reply Port - not used
    replyPort = 0x00000000
    message += struct.pack('>I', replyPort)

    #8b
    # max callsign lenght 7
    callsign = "{:<7}".format(callsign)
    print("'"+callsign+"'")
    callsign = bytes(callsign, encoding='raw_unicode_escape')
    message += callsign #[:7]
    message += __TT1# zero terminated

    #CHAT max 256b
    #chat=bytes(CHAT, encoding='raw_unicode_escape')
    #message += chat[:256]
    #message += __TT2 # zero terminated

    #96b
    #Model max length = 96b ie. "/model/foo/aero.xml" Name of the aircraft model.
    model = bytes(model, encoding='raw_unicode_escape')
    message += model[:96]
    for i in range(1, 96 -len(model)):
        message += b'\x00'
    message += __TT1 # zero terminated

    #8b
    #time epoch time Time when this packet was generated.
    NOW = time.time()
    #print('Time %s' % NOW)
    now = struct.pack('>d',  NOW)
    message += now

    #8b
    #lag
    lag=0x02
    message += struct.pack('>d', lag)

    #24b
    #PositionMsg https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
    POS = pymap3d.geodetic2ecef(aircraft_lat, aircraft_long, aircraft_height)
    # i.e. 6378137.0 -> b'AXT\xa6@\x00\x00\x00' # # # -> binascii.hexlify equals b'415854a640000000'
    pos_lat = POS[0]
    pos_long = POS[1]
    pos_height = POS[2]
    #print('pos_lat=%s, pos_long=%s, pos_height= %s' % ( POS[0], POS[1], POS[2]))
    
    POS_LAT = struct.pack('>d', POS[0])
    POS_LONG = struct.pack('>d', POS[1]) 
    POS_HEIGHT = struct.pack('>d', POS[2]) 
    
    POS_LAT1 = struct.pack('>d', 0)
    POS_LONG1 = struct.pack('>d', 0) 
    POS_HEIGHT1 = struct.pack('>d', 0) 

    message += POS_LAT
    message += POS_LONG
    message += POS_HEIGHT

    # 12b
    #hdgorientation float[3] Orientation wrt the earth centered frame, stored in the angle axis representation where the angle is coded into the axis length.
    ORIENT = _ac_rotate(aircraft_lat, aircraft_long, aircraft_height, angle_x=angle_x, angle_y=angle_y, angle_z=angle_z, vel=0, vs=0, turnrate=0)
    ##print(f"ORIENT={ORIENT}")

    orient_x = ORIENT[0]
    orient_y = ORIENT[1]
    orient_z=  ORIENT[2]
    
    #print("angle_x= %s, angle_y = %s, angle_z = %s"  % (angle_x, angle_y, angle_z))
    #print("orient_x = %s, orient_y = %s, orient_z = %s" % (orient_x, orient_y, orient_z))
    #print("aircraft_lat= %s, aircraft_long = %s, aircraft_height = %s"  % (aircraft_lat, aircraft_long, aircraft_height))

    message += struct.pack ('>f', orient_x)
    message += struct.pack ('>f', orient_y)
    message += struct.pack ('>f', orient_z)

    # 12b
    #linearVel float[3] Linear velocity wrt the earth centered frame measured in the earth centered frame.
    linearVel_X = 0
    linearVel_Y = 0
    linearVel_Z = 0
    message += struct.pack('>I', linearVel_X)
    message += struct.pack('>I', linearVel_Y)
    message += struct.pack('>I', linearVel_Z)


    # 12b
    #angularVel float[3] Angular velocity wrt the earth centered frame measured in the earth centered frame.
    angularVel_X = 0
    angularVel_Y = 0
    angularVel_Z = 0
    message += struct.pack('>I', angularVel_X)
    message += struct.pack('>I', angularVel_Y)
    message += struct.pack('>I', angularVel_Z)


    # 12b
    #linearAccel float[3] Linear acceleration wrt the earth centered frame measured in the earth centered frame.
    linearAccel_X = 0
    linearAccel_Y = 0
    linearAccel_Z = 0
    message += struct.pack('>I', linearAccel_X)
    message += struct.pack('>I', linearAccel_Y)
    message += struct.pack('>I', linearAccel_Z)

    # 12b
    #angularAccel float[3] Angular acceleration wrt the earth centered frame measured in the earth centered frame.
    angularAccel_X = 0
    angularAccel_Y = 0
    angularAccel_Z = 0
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

    #  position wrt the earth centered frame
    #SGVec3d position;
    # T_PositionMsg https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
    #position   double[3] Position wrt the earth centered frame.
    #message += struct.pack ('>Q', 0xc154c68286FFFFFF)
    # message += struct.pack('>Q', 0xc143531d84FFFFFF)
    #message += struct.pack('>Q', 0x3ffacaa333FFFFFF)

    message += POS_LAT1
    message += POS_LONG1
    message += POS_HEIGHT1
    
    #  orientation wrt the earth centered frame
    #SGQuatf orientation;
    #hdgorientation float[3] Orientation wrt the earth centered frame, stored in the angle axis representation where the angle is coded into the axis length.
    ori_x=0
    ori_y=0
    ori_z=0
    message += struct.pack('>f', ori_x)
    message += struct.pack('>f', ori_y)
    message += struct.pack('>f', ori_z)

    #  linear velocity wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f linearVel;

    #linearVel float[3] Linear velocity wrt the earth centered frame measured in the earth centered frame.
    linearVel_X = 0
    linearVel_Y = 0
    linearVel_Z = 0
    message += struct.pack('>f', linearVel_X)
    message += struct.pack('>f', linearVel_Y)
    message += struct.pack('>f', linearVel_Z)


    #  angular velocity wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f angularVel;
    #angularVel float[3] Angular velocity wrt the earth centered frame measured in the earth centered frame.
    angularVel_X = 0
    angularVel_Y = 0
    angularVel_Z = 0
    message += struct.pack('>f', angularVel_X)
    message += struct.pack('>f', angularVel_Y)
    message += struct.pack('>f', angularVel_Z)


    #  linear acceleration wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f linearAccel;
    #linearAccel float[3] Linear acceleration wrt the earth centered frame measured in the earth centered frame.
    linearAccel_X = 0
    linearAccel_Y = 0
    linearAccel_Z = 0
    message += struct.pack('>f', linearAccel_X)
    message += struct.pack('>f', linearAccel_Y)
    message += struct.pack('>f', linearAccel_Z)


    #  angular acceleration wrt the earth centered frame measured in
    #  the earth centered frame
    #SGVec3f angularAccel;
    #angularAccel float[3] Angular acceleration wrt the earth centered frame measured in the earth centered frame.
    angularAccel_X = 0
    angularAccel_Y = 0
    angularAccel_Z = 0
    message += struct.pack('>f', angularAccel_X)
    message += struct.pack('>f', angularAccel_Y)
    message += struct.pack('>f', angularAccel_Z)


    return message

if __name__ == "__main__":
    pass

