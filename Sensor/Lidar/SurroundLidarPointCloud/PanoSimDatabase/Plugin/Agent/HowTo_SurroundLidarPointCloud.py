from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    measurements_per_rotation, number_of_beams = 360, 32
    OutputFormat = 'time@i,%d@[,x@f,y@f,z@f,intensity@f' % (measurements_per_rotation * number_of_beams)
    userData['sensor'] = BusAccessor(userData['busId'], 'SurroundLidarPointCloudSensor.0', OutputFormat)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Surround Lidar Point Cloud')
    userData['ax'] = plt.axes(projection='3d')

def ModelOutput(userData):
    timestamp, count = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.cla()
        plt.title('Sensor: Surround Lidar Point Cloud')
        userData['ax'].set_xlabel('X')
        userData['ax'].set_ylabel('Y')
        userData['ax'].set_zlabel('Z')
        userData['ax'].set_xlim((-100, 100))
        userData['ax'].set_ylim((-100, 100))
        userData['ax'].set_zlim((-100, 100))
        points = np.frombuffer(userData['sensor'].getBus()[8:], dtype=np.float32).reshape((count, 4))
        indices = np.argwhere(np.all(points[:,3] == 0))
        points = np.delete(points, indices, axis=0)
        userData['ax'].scatter(points[:,0], points[:,1], points[:,2], s=1, c='red')
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
