from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString

def ModelStart(userData):
    Format = 'time@i,lane@i,station@d,lateral@d,internal@b,nextJunction@i'
    userData['ego_traffic'] = BusAccessor(userData['busId'], 'ego_traffic', Format)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Bus: ego_traffic')

def ModelOutput(userData):
    timestamp, lane, station, lateral, internal, nextJunction = userData['ego_traffic'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.axes().set_aspect('equal')
        plt.title('Bus: ego_traffic')
        plt.xlabel('X')
        plt.ylabel('Y')
        shape = getLaneShape(lane)
        pts = np.array(shape)
        plt.plot(pts[:,0], pts[:,1], c='m' if internal == 1 else 'b', label='lane:{}'.format(lane))
        line = LineString(shape)
        point = line.interpolate(station)
        ego_x, ego_y = point.x, point.y
        plt.xlim((ego_x-100, ego_x+100))
        plt.ylim((ego_y-100, ego_y+100))
        plt.scatter(x=ego_x, y=ego_y, c='red', label='ego')
        plt.text(x=ego_x+2, y=ego_y+2, s='({:.2f},{:.2f})'.format(station, lateral), color='g')
        pts = np.array(getJunctionShape(nextJunction))
        plt.plot(pts[:,0], pts[:,1], c='y', label='junciton:{}'.format(nextJunction))
        plt.legend()
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
