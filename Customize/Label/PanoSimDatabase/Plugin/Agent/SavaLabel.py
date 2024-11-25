from DataInterfacePython import *
import csv
import math
import glob
import os
import numpy as np
from Demo_FreeDriving_Source import CameraTools
import xml.etree.cElementTree as ET
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import sys
import pickle

Ratio_Format = "time@i,256@[,id@i,type@b,occlu@d"
OUTPUT_FORMAT = "Timestamp@i,256@[,OBJ_ID@i,OBJ_Class@b,OBJ_X@d,OBJ_Y@d,OBJ_Z@d,OBJ_Velocity@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d"
IMUFormat = "Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Yaw@d,Pitch@d,Roll@d"
EGO="time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d"
TRAFFIC_FORMAT = "time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f"
last_time = 0
light_list = []
cross_list = []
sign_list = []
traffic_data = {}

def ModelStart(userData):
    global light_list, cross_list
    BusID=userData["busId"]
    userData["bus"]=BusAccessor(BusID,userData["parameters"]["CameraName"],OUTPUT_FORMAT)
    userData["IMU"] = BusAccessor(BusID, "IMU.0", IMUFormat)
    userData["EGO"]=BusAccessor(BusID,"ego",EGO)
    userData["Position"] = userData["parameters"]["Position"]
    userData["traffic"] = DoubleBusReader(userData["busId"], "traffic", TRAFFIC_FORMAT)
    dic = {}

    obj_num = userData["parameters"]["CameraName"].split('.')[-1]
    CameraName = "MonoCameraSensor." + obj_num
    occlu_name = "MonoDetector_OcclusionRatio." + obj_num
    userData["ratio"] = BusAccessor(BusID, occlu_name, Ratio_Format)

    xmlpath = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + userData["outputPath"].split('/')[
        0] + '/Temp.experiment.xml'
    userData["field"] = userData["outputPath"].split('/')[0].split("_")[0]
    tree = ET.parse(xmlpath)
    root = tree.getroot()
    tem = userData["parameters"]["CameraName"].split('.')[0]
    num = int(userData["parameters"]["CameraName"].split('.')[-1])
    n = 0

    for sensor in root.iter('Sensor'):
        if tem in sensor.attrib['link']:
            if n == num:
                dic["resolution"] = (int(sensor.attrib['resolutionwidth']), int(sensor.attrib['resolutionheight']))
                dic["fov"] = float(sensor.attrib["FovHorizontal"])
                dic["position"] = (float(sensor.attrib['X']), float(sensor.attrib['Y']),float(sensor.attrib['Z']))
                dic["rotation"] = (-math.radians(float(sensor.attrib['Yaw'])), -math.radians(float(sensor.attrib['Pitch'])),
                                   -math.radians(float(sensor.attrib['Roll'])))
                break
            else:
                n = n + 1

    userData["width"] = dic["resolution"][0]
    userData["height"] = dic["resolution"][1]
    userData["Sensor_Yaw"] = -dic["rotation"][0]
    userData["Sensor_Coor"] = [dic["position"][0],dic["position"][1], dic["position"][2]]
    for light in root.iter('TrafficLight'):
        if light.attrib['type'] == "2" or light.attrib['type'] == "1":
            light_list.append([float(light.attrib['X']), float(light.attrib['Y']), float(light.attrib['Yaw'])])

    for cross in root.iter('Crosswalk'):
        if cross.attrib['type'] == "3":
            cross_list.append([float(cross.attrib['X']), float(cross.attrib['Y']), float(cross.attrib['Yaw']),
                               float(cross.attrib['Length']), float(cross.attrib['Width'])])

    for sign in root.iter('TrafficSign'):
        sign_list.append([float(sign.attrib['X']), float(sign.attrib['Y']), float(sign.attrib['Yaw']), int(sign.attrib['type'])])

    cameraTools = CameraTools.CameraTools()
    cameraTools.resetCameraPrameter(userData["width"], userData["height"], dic["fov"])
    cameraTools.resetCamRT(dic["position"], dic["rotation"])
    cameraTools.resetCarRT((3.43, -45.03, 0.5), (0, 0, 0))
    userData["cameraTools"] = cameraTools

    userData["name"] = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + userData["outputPath"] + "/" + userData["parameters"]["CameraName"]
    if os.path.exists(userData["name"]) == False:
        os.mkdir(userData["name"])
    log_path = userData["name"] + '/' + userData["outputPath"].split('/')[
        0] + "_" +userData["Position"] + '.txt'
    file = open(log_path, 'w').close()

def ModelOutput(userData):
    global last_time, light_list, cross_list, traffic_data
    ego_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = userData["EGO"].readHeader()
    (Timestamp, ACC_X, ACC_Y, ACC_Z, Gyro_X, Gyro_Y, Gyro_Z, Yaw, Pitch, Roll) = userData["IMU"].readHeader()

    log_path = userData["name"] + '/' + userData["outputPath"].split('/')[
        0] + "_" +userData["Position"] + '.txt'

    object_list = np.full((1, 2048), 0)
    ts1, number1 = userData["ratio"].readHeader()
    LIST = []

    for mn in range(number1):
        index,cla,occlu = userData["ratio"].readBody(mn)
        if cla != 3:
            if occlu <= 0.75 and occlu >= 0:
                object_list[0,index] = 1
                LIST.append([index, cla, 0, 0, 0, 0, 0, 0, 0, abs(occlu)])
        elif cla == 3:
            if occlu >= 0 and occlu<=0.75:
                object_list[0, index] = 1
                LIST.append([index, 2, 0, 0, 0, 0, 0, 0, 0, occlu])

    Traffuc_time, Traffic_num = userData["traffic"].getReader(userData["time"]).readHeader()

    if len(traffic_data) > 0 and Timestamp == traffic_data["timestamp"]:
        traffic_data.clear()

    if Traffuc_time >= 500:
        if len(traffic_data) == 0:
            if Traffuc_time % 100 >= 60 and Traffuc_time % 100 <= 70:
                traffic_data["timestamp"] = math.floor(Traffuc_time / 100) * 100
                traffic_data["number"] = Traffic_num
                data_list = [[] for i in range(Traffic_num)]
                for j in range(Traffic_num):
                    id, type, shape, tra_x, tra_y, tra_z, tra_yaw, tra_pitch, tra_roll, tra_speed = userData[
                        "traffic"].getReader(
                        userData["time"]).readBody(j)
                    data_list[j] = [id, type, shape, tra_x, tra_y, tra_z, tra_yaw, tra_pitch, tra_roll, tra_speed]
                traffic_data["data"] = data_list

    other_vehicle_car = [86, 87, 88, 11140, 11141, 11142]
    other_vehicle_rider = [140, 141, 142, 143, 144]
    other_large_vehicle = [20, 21, 108, 109, 11117, 11120, 11123, 124, 126, 130]

    if Timestamp > 500 and Timestamp - last_time >= int(userData["parameters"]["TimeStep"]) and len(traffic_data) > 0 and Timestamp == traffic_data["timestamp"]:
        last_time = Timestamp
        userData["cameraTools"].resetCarRT((ego_x, ego_y, ego_z), (ego_yaw, ego_pitch, ego_roll))
        bbox = []
        for i in range(len(LIST)):
            index, cla, _, _, _, _, _, _, _, occlu = LIST[i]
            cilp_data = [[], []]
            # car
            if cla == 0:
                for j in range(len(traffic_data["data"])):
                    id, type, shape, tra_x, tra_y, tra_z, tra_yaw, tra_pitch, tra_roll, tra_speed = traffic_data["data"][j]
                    if index == id:
                        vetex_yaw = tra_yaw
                        if tra_yaw < 0:
                            tra_yaw += 360
                        elif tra_yaw > 360:
                            tra_yaw -= 360
                        tra_yaw -= 90
                        if tra_yaw < 0:
                            tra_yaw += 360
                        yaw1 = ego_yaw - userData["Sensor_Yaw"]
                        if yaw1 <= 0:
                            yaw1 = - yaw1
                        else:
                            yaw1 = 2 * 3.1415926 - yaw1

                        SIZE2 = getObjectSize(object_type(type), shape)
                        (sizex, sizey, sizez) = SIZE2
                        vertexs = getObjectVertex(object_type(type), shape, tra_x, tra_y, tra_z, vetex_yaw, tra_pitch, tra_roll)

                        subtype = getObjectSubtype(object_type(type), shape)
                        label_id = 2
                        if subtype == object_subtype.Car:
                            label_id = 2
                        elif subtype == object_subtype.OtherVehicle:
                            if shape in other_vehicle_car:
                                label_id = 2
                            elif shape in other_vehicle_rider:
                                label_id = 8
                            elif shape in other_large_vehicle:
                                label_id = 6
                            elif shape == 11145:
                                label_id = 14
                            else:
                                label_id = 3
                        elif subtype == object_subtype.Bus:
                            label_id = 4
                        elif subtype == object_subtype.Pedestrian:
                            label_id = 9
                        elif subtype == object_subtype.Van:
                            label_id = 6

                        traffic_vertex = np.ones((4, 8))
                        for g in range(8):
                            traffic_vertex[:3, g] = np.array(vertexs[g+1])

                        clip = userData["cameraTools"].getPixelSpacePoints(traffic_vertex)
                        clip = clip.astype(np.int32)

                        QUA = quaternion(tra_yaw/180*math.pi-yaw1,tra_pitch/180*math.pi-ego_pitch,tra_roll/180*math.pi-ego_roll)

                        center_vertex = np.ones((4, 1))
                        p39 = vertexs[0]
                        center_vertex[:3, 0] = np.array(p39)
                        coor = userData["cameraTools"].getCoordinate(center_vertex)
                        dx = coor[0][0]
                        dy = coor[1][0]

                        for cilp_num in range(8):
                            cilp_data[0].append(clip[0][cilp_num])
                            cilp_data[1].append(clip[1][cilp_num])

                        if len(cilp_data[0]) > 0 and len(cilp_data[1]) > 0:
                            point = [min(cilp_data[0]), min(cilp_data[1]), max(cilp_data[0]), max(cilp_data[1])]
                        else:
                            continue
                        print(point)

                        if point[0] < 0:
                            point[0] = 0
                        if point[1] < 0:
                            point[1] = 0
                        if point[2] >= userData["width"]:
                            point[2] = userData["width"] - 1
                        if point[3] >= userData["height"]:
                            point[3] = userData["height"] - 1
                        if point[0]>=userData["width"] or point[1]>=userData["height"] or point[2]<0 or point[3]<0:
                            continue

                        if (point[2] - point[0]) < 32 or (point[3] - point[1]) < 32:
                            continue

                        bbox.append(
                            [int(label_id), round(point[0], 3), round(point[1], 3), round(point[2] - point[0], 3),
                             round(point[3] - point[1], 3),
                             round(dx, 3), round(dy, 3), round(tra_z - userData["Sensor_Coor"][2], 3), round(tra_yaw-yaw1*180/math.pi,3),
                             round(tra_pitch - ego_pitch * 180 / math.pi, 3),
                             round(tra_roll - ego_roll * 180 / math.pi, 3),
                             round(sizex, 3), round(sizey, 3), round(sizez, 3), int(Timestamp), round(occlu, 3),
                             round(QUA[0], 3), round(QUA[1], 3), round(QUA[2], 3), round(QUA[3], 3), index])

        with open(log_path, 'ab+') as txt_file:
            pickle.dump(bbox, txt_file)

        if Timestamp >= 60000:
            txt_file.close()
            stopSimulation()

def ModelTerminate(userData):
    pass


def quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    return [w, x, y, z]

def sortF(i):
    range = math.sqrt(i[2]*i[2] + i[3] * i[3] + i[4] * i[4])
    return range

