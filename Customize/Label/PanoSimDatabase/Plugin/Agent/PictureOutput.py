import numpy as np
import imageio
import os
from DataInterfacePython import *
import shutil
import xml.etree.cElementTree as ET
from _thread import start_new_thread
import cv2
IMUFormat = "Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Yaw@d,Pitch@d,Roll@d"
def ModelStart(userData):

    userData["last"] = 0
    userData["name"] = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + userData["outputPath"] + "/" + userData["parameters"]["CameraName"]
    os.mkdir(userData["name"])
    xmlpath = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + userData["outputPath"].split('/')[
        0] + '/Temp.experiment.xml'
    try:
        tree = ET.parse(xmlpath)

    except:
        xmlpath = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + '\\'.join(userData["outputPath"].split('/')[:-1]) + '.experiment.xml'
        tree = ET.parse(xmlpath)
    root = tree.getroot()

    dic = {}
    n = 0
    tem = userData["parameters"]["CameraName"].split('.')[0]
    num = int(userData["parameters"]["CameraName"].split('.')[-1])
    for sensor in root.iter('Sensor'):
        if tem in sensor.attrib['link']:
            if n == num:
                dic["resolution"] = (int(sensor.attrib['resolutionwidth']), int(sensor.attrib['resolutionheight']))
                break
            else:
                n = n + 1
    userData["width"] = dic["resolution"][0]
    userData["height"] = dic["resolution"][1]
    userData["timestep"] = int(userData["parameters"]["TimeStep"])
    userData["bus"] = BusAccessor(userData["busId"], userData["parameters"]["CameraName"],
                                  "time@i,%d@[,r@b,g@b,b@b" % (dic["resolution"][0] *dic["resolution"][1]))
    userData["IMU"] = BusAccessor(userData["busId"], "IMU.0", IMUFormat)
    userData["Position"] = userData["parameters"]["Position"]
def ModelOutput(userData):
    ts, _ = userData["bus"].readHeader()
    (Timestamp, ACC_X, ACC_Y, ACC_Z, Gyro_X, Gyro_Y, Gyro_Z, Yaw, Pitch, Roll) = userData["IMU"].readHeader()
    if Timestamp - userData["last"] >= int(userData["parameters"]["TimeStep"]) and Timestamp > 500:
        userData["last"] = Timestamp

        img = np.frombuffer(userData["bus"].getBus()[8:], dtype=np.uint8).reshape((userData["height"], userData["width"], 3))

        name = userData["name"] + "/" + userData["parameters"]["CameraName"].split('.')[0] + "_" + userData["Position"] + "_" + str(Timestamp) + ".png"

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(name, img)

        if Timestamp >= 60100:
            stopSimulation()
def ModelTerminate(userData):
    # userData["writer"].close()
    pass


