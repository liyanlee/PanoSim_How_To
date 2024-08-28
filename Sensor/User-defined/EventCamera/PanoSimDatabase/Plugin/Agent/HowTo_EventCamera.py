from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    OutputFormat = 'time@i,%d@[,x@i,y@i,p@i,t@d' % (240 * 180 * 10)
    userData['sensor'] = BusAccessor(userData['busId'], 'EventCamera.0', OutputFormat)
    userData['last_time'] = 0
    userData['DataType'] = np.dtype([('x', np.int), ('y', np.int), ('p', np.int), ('t', np.uint64)])
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Event Camera')

def ModelOutput(userData):
    timestamp, _ = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        data = np.frombuffer(userData['sensor'].getBus()[8:(8+_*20)], dtype=userData['DataType'])
        buffer1 = np.zeros((_,3),dtype=np.int)
        buffer1[:,0] = (1 - data['p']) * 255
        buffer1[:,2] = data['p'] * 255
        buffer2 = np.zeros((180, 240, 3))
        buffer2[data['x'], data['y']] = buffer1
        plt.clf()
        plt.title('Sensor: Event Camera')
        plt.imshow(buffer2.astype('uint8'))
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
