from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    userData['sensor'] = BusAccessor(userData['busId'], 'FreeSpacePerception.0', 'time@i,3600@[,x@d,y@d')
    userData['last_time'] = 0
    userData['img'] = np.ones((500,500), np.uint8) * 255
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Free Space Perception')

def ModelOutput(userData):
    timestamp, _ = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.title('Sensor: Free Space Perception')
        plt.axis('off')
        userData['img'][:,:] = 255
        points = np.frombuffer(userData['sensor'].getBus()[8:], dtype=np.float64).reshape((3600, 2)).T
        points = (-points * 24)[:,:] + 249.5
        userData['img'][points[0,:].astype(np.int), points[1,:].astype(np.int)] = 0
        plt.imshow(userData['img'])
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
