from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    OutputFormat = 'time@i,%d@[,r@b,g@b,b@b' % (1280 * 720)
    userData['sensor'] = BusAccessor(userData['busId'], 'DepthmapPerception.0', OutputFormat)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Depthmap Perception')

def ModelOutput(userData):
    timestamp, _ = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.title('Sensor: Depthmap Perception')
        rgb = np.frombuffer(userData['sensor'].getBus()[8:], dtype=np.uint8).reshape((720, 1280, 3))
        rgb_float = np.array(rgb, dtype='f')
        gray = (rgb_float[:,:,0] + rgb_float[:,:,1]/255.0 + rgb_float[:,:,2]/65025.0)/255
        plt.imshow(gray)
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
