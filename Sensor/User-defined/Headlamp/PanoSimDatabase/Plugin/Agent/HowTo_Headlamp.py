import socket
import numpy as np
import math
import time
import struct
import cv2
import os
from shapely.geometry import LineString, Point, Polygon
import matplotlib.pyplot as plt
from DataInterfacePython import *

LowBeam_LIGHT_FORMAT = "time@i,pitch@d,yaw@d,intensity@d"
PROJECTION_LIGHT_FORMAT = "time@i,valid@b,type@b,pitch@d,yaw@d,25600@[,intensity@b"
PROJECTION_LIGHT_OBJECT_FORMAT = "time@i,10@[,x1@i,y1@i,x2@i,y2@i"
ADB_LIGHT_FORMAT = "time@i,pitch@d,yaw@d,18@[,i@d"

def ModelStart(userData):
    userData["projectionLight_right"] = BusAccessor(userData["busId"],"ProjectionLight.0",PROJECTION_LIGHT_FORMAT)
    userData["projectionLight_left"] = BusAccessor(userData["busId"],"ProjectionLight.1",PROJECTION_LIGHT_FORMAT)
    userData["projectionLight_Object_right"] = BusAccessor(userData["busId"], "ProjectionObject.0", PROJECTION_LIGHT_OBJECT_FORMAT)
    userData["projectionLight_Object_left"] = BusAccessor(userData["busId"], "ProjectionObject.1", PROJECTION_LIGHT_OBJECT_FORMAT)
    
    userData["lowBeam_right"] = BusAccessor(userData["busId"], "LowBeam.0", LowBeam_LIGHT_FORMAT)
    userData["lowBeam_left"] = BusAccessor(userData["busId"], "LowBeam.1", LowBeam_LIGHT_FORMAT)

    userData["adbLight_right"] = BusAccessor(userData["busId"], "ADB.0", ADB_LIGHT_FORMAT)
    userData["adbLight_left"] = BusAccessor(userData["busId"], "ADB.1", ADB_LIGHT_FORMAT)

def ModelOutput(userData):
    time = userData["time"]
    if time > 0 and time < 3000:
        ControlProjectionLight(userData, 1, b'\x00',b'\x00')
        ControlProjectionLight(userData, 0, b'\x00',b'\x00')

        ControlLowBeam(userData, 0, 0, 0, struct.pack('d', 0.0))
        ControlLowBeam(userData, 1, 0, 0, struct.pack('d', 0.0))

        swiths_right =  [0.0] * 18
        swiths_left =  [0.0] * 18
        format_swiths = f'{len(swiths_right)}d'
        ControlADBLight(userData, 0, 0, 0, struct.pack(format_swiths, *swiths_right))
        ControlADBLight(userData, 1, 0, 0, struct.pack(format_swiths, *swiths_left))
    elif time > 3000 and time < 8000:
        # 打开近光灯
        if time > 3000:
            ControlProjectionLight(userData, 1, b'\x01', b'\x00')
            ControlProjectionLight(userData, 0, b'\x01', b'\x00')
        if time > 5000:
            ControlLowBeam(userData, 0, 0, 0, struct.pack('d', 1.0))
            ControlLowBeam(userData, 1, 0, 0, struct.pack('d', 1.0))

        #关闭远光灯
        swiths = [0.0] * 18
        format_swiths = f'{len(swiths)}d'
        ControlADBLight(userData, 0, 0, 0, struct.pack(format_swiths, *swiths))
        ControlADBLight(userData, 1, 0, 0, struct.pack(format_swiths, *swiths))
    elif time > 8000 and time < 25000:
        # 打开投影灯，投影人行横道线
        ControlProjectionLightSignal(userData, 1, b'\x01', "projectionLight_RoadSigns_left.bmp")
        ControlProjectionLightSignal(userData, 0, b'\x01', "projectionLight_RoadSigns_right.bmp")

        ControlLowBeam(userData, 0, 0, 0, struct.pack('d', 0.5))
        ControlLowBeam(userData, 1, 0, 0, struct.pack('d', 0.5))

        swiths = [0.0] * 18
        format_swiths = f'{len(swiths)}d'
        ControlADBLight(userData, 0, 0, 0, struct.pack(format_swiths, *swiths))
        ControlADBLight(userData, 1, 0, 0, struct.pack(format_swiths, *swiths))
        # 行人过马路
    elif time > 28000 and time < 35000:
        ControlLowBeam(userData, 0, 0, 0, struct.pack('d', 1.0))
        ControlLowBeam(userData, 1, 0, 0, struct.pack('d', 1.0))

        # 打开远光灯
        if time > 28000:
            ControlProjectionLight(userData, 1, b'\x02',b'\x00')
            ControlProjectionLight(userData, 0, b'\x02',b'\x00')
        if time > 32000:
            swiths_right = [1.0] * 18
            swiths_left = [1.0] * 18
            format_swiths = f'{len(swiths_right)}d'
            ControlADBLight(userData, 0, 0, 0, struct.pack(format_swiths, *swiths_right))
            ControlADBLight(userData, 1, 0, 0, struct.pack(format_swiths, *swiths_left))
    elif time > 35000:
        ControlLowBeam(userData, 0, 0, 0, struct.pack('d', 1.0))
        ControlLowBeam(userData, 1, 0, 0, struct.pack('d', 1.0))
        ControlProjectionLight(userData, 1, b'\x02', b'\x01')
        ControlProjectionLight(userData, 0, b'\x02', b'\x01')
        # 关闭扇区
        if time > 37000:
            swiths_right = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            swiths_left = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            format_swiths = f'{len(swiths_right)}d'
            ControlADBLight(userData, 0, 0, 0, struct.pack(format_swiths, *swiths_right))
            ControlADBLight(userData, 1, 0, 0, struct.pack(format_swiths, *swiths_left))

        if time > 40000:
            ControlBlackArea(userData, 1)
            ControlBlackArea(userData, 0)

def ControlADBLight(userData,index,pitch,yaw,switch):
    ts = userData["time"]

    bytes_time = struct.pack('i',ts)
    bytes_pitch = struct.pack('d',pitch)
    bytes_yaw = struct.pack('d',yaw)
    bytes_size = struct.pack('i',18)

    if index == 0:
        userData["adbLight_right"].getBus()[:] = bytes_time + bytes_pitch + bytes_yaw + bytes_size + switch
    else:
        userData["adbLight_left"].getBus()[:] = bytes_time + bytes_pitch + bytes_yaw + bytes_size + switch

def ControlProjectionLight(userData,index,bytes_type,bytes_valid):
    ts = userData["time"]

    size = 320 * 80
    bytes_size = struct.pack('i', size)
    bytes_time = struct.pack('i', ts)
    bytes_pitch = struct.pack('d', 0)
    bytes_yaw = struct.pack('d', 0)

    byte_intensity = b'\x00' * size

    if index == 0:
        userData["projectionLight_right"].getBus()[:] = bytes_time + bytes_valid + bytes_type + bytes_pitch + bytes_yaw + bytes_size + byte_intensity
    else:
        userData["projectionLight_left"].getBus()[:] = bytes_time + bytes_valid + bytes_type + bytes_pitch + bytes_yaw + bytes_size + byte_intensity

def ControlProjectionLightSignal(userData,index,bytes_valid,sign_path):
    ts = userData["time"]

    current_directory = os.path.dirname(os.path.abspath(__file__))
    sign_path = os.path.join(current_directory, sign_path)

    img_r = cv2.imread(sign_path)
    img = cv2.flip(img_r, 0)
    img = cv2.resize(img, (320, 80))
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    size = 320 * 80
    bytes_size = struct.pack('i', size)
    bytes_time = struct.pack('i', ts)
    bytes_pitch = struct.pack('d', 0)
    bytes_yaw = struct.pack('d', 0)
    bytes_type = b'\x01'

    byte_intensity = img_gray.tobytes()

    if index == 0:
        userData["projectionLight_right"].getBus()[:] = bytes_time + bytes_valid + bytes_type + bytes_pitch + bytes_yaw + bytes_size + byte_intensity
    else:
        userData["projectionLight_left"].getBus()[:] = bytes_time + bytes_valid + bytes_type + bytes_pitch + bytes_yaw + bytes_size + byte_intensity

def ControlLowBeam(userData,index,pitch,yaw,intensity):
    ts = userData["time"]
    bytes_time = struct.pack('i',ts)
    bytes_pitch = struct.pack('d', pitch)
    bytes_yaw = struct.pack('d', yaw)

    if index == 0:
        userData["lowBeam_right"].getBus()[:] = bytes_time + bytes_pitch + bytes_yaw + intensity
    else:
        userData["lowBeam_left"].getBus()[:] = bytes_time + bytes_pitch + bytes_yaw + intensity

def ControlBlackArea(userData,index):
    ts = userData["time"]
    bytes_time = struct.pack('i', ts)
    bytes_widh = struct.pack('i',1)
    area = [70,0,110,80]
    packed_area  = struct.pack(f'{len(area)}i',*area)
    padding_count = (10 - 1) * 4
    padding_data = [0] * padding_count
    padding_bytes = struct.pack(f'{padding_count}i', *padding_data)
    bytes_area = packed_area + padding_bytes

    if index == 0:
        userData["projectionLight_Object_right"].getBus()[:] = bytes_time + bytes_widh + bytes_area
    else:
        userData["projectionLight_Object_left"].getBus()[:] = bytes_time + bytes_widh + bytes_area

def ModelTerminate(userData):
    pass
