from DataInterfacePython import *
import os
import math
import imageio
import numpy as np
from pathlib import Path

def ModelStart(userData):
    width, height = 320, 240
    userData['camera0'] = BusAccessor(userData['busId'], 'MonoCameraSensor.0', 'time@i,%d@[,r@b,g@b,b@b' % (width * height))
    userData['camera1'] = BusAccessor(userData['busId'], 'MonoCameraSensor.1', 'time@i,%d@[,r@b,g@b,b@b' % (width * height))
    userData['floder0'] = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + userData["outputPath"] + "/camera0/"
    userData['floder1'] = os.environ["PanoSimDatabaseHome"] + "/Experiment/" + userData["outputPath"] + "/camera1/"
    if not Path(userData["floder0"]).exists():
        os.mkdir(userData["floder0"])
    if not Path(userData["floder1"]).exists():
        os.mkdir(userData["floder1"])

def ModelOutput(userData):
    ts = userData['time']
    gap = 1000.0 / 30
    width, height = 320, 240
    camera0_ok, camera1_ok = False, False
    if math.floor(ts / gap) != math.floor((ts - 10) / gap):
        while True:
            if userData['camera0'].readHeader()[0] == ts and not camera0_ok:
                file = userData['floder0'] + str(ts) + '.png'
                image = np.frombuffer(userData["camera0"].getBus()[8:], dtype=np.uint8).reshape((height, width, 3))
                imageio.imsave(file , image)
                camera0_ok = True
            if userData['camera1'].readHeader()[0] == ts and not camera1_ok:
                file = userData['floder1'] + str(ts) + '.png'
                image = np.frombuffer(userData["camera1"].getBus()[8:], dtype=np.uint8).reshape((height, width, 3))
                imageio.imsave(file , image)
                camera1_ok = True
            if camera0_ok and camera1_ok:
                break

def ModelTerminate(userData):
    pass
