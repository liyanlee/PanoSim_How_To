from DataInterfacePython import *
from pyproj import Proj
import matplotlib.pyplot as plt

def ModelStart(userData):
    Format = 'Timestamp@i,Longitude@d,Latitude@d,Altitude@d,Heading@d,Velocity@d'
    userData['sensor'] = BusAccessor(userData["busId"], 'GNSSHIFI.0', Format)
    userData['last_time'] = 0
    userData["Projection"] = Proj(proj='utm', zone=51, ellps='WGS84', south=False, north=True, errcheck=True)
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: GNSSHIFI')
    plt.title('Sensor: GNSSHIFI')
    plt.axes().set_aspect('equal')
    plt.xlabel('X')
    plt.ylabel('Y')

def ModelOutput(userData):
    timestamp, longitude, latitude, _, _, _ = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        x, y = userData["Projection"](longitude, latitude, inverse=False)
        plt.scatter(x=x, y=y, c='blue')
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
