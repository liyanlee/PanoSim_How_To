from DataInterfacePython import *
import numpy as np
from pyproj import Proj
import matplotlib.pyplot as plt

def ModelStart(userData):
    userData['bus_ego'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    OutputFormat = 'time@i,Longitude@d,Latitude@d,Altitude@d,Heading@d,Velocity@d,UtcTime@d'
    userData['bus_output'] = BusAccessor(userData['busId'], 'Customize_Sensor_Location', OutputFormat)
    userData['utc_origin'] = getUtcOrigin()
    projection_str, offset_x, offset_y = getMapProjection()
    userData['proj'] = Proj(projection_str, preserve_units=False)
    userData['offset_x'] = offset_x
    userData['offset_y'] = offset_y
    plt.ion()
    fig, (userData['xy'], userData['longitude_latitude']) = plt.subplots(1, 2)
    fig.canvas.set_window_title('HowTo Customize Sensor: Location')
    fig.suptitle('Sensor: Location')
    userData['timestamps'], userData['x'], userData['y'] = [], [], []
    userData['longitude'], userData['latitude'] = [], []

def ModelOutput(userData):
    ego_time, ego_x, ego_y, ego_z, ego_yaw, _, _, ego_speed = userData['bus_ego'].readHeader()
    longitude, latitude = userData['proj'](ego_x - userData['offset_x'], ego_y - userData['offset_y'], inverse=True)
    heading = np.degrees(np.pi / 2 - ego_yaw)
    utc_time = userData['utc_origin'] + ego_time / 1000.0
    userData['bus_output'].writeHeader(*(ego_time, longitude, latitude, ego_z, heading, ego_speed / 3.6, utc_time))
    userData['timestamps'].append(ego_time)
    userData['x'].append(ego_x)
    userData['y'].append(ego_y)
    userData['longitude'].append(longitude)
    userData['latitude'].append(latitude)
    userData['xy'].set_title('x,y')
    userData['xy'].set_xlabel('time(ms)')
    userData['xy'].plot(userData['timestamps'], userData['x'], c='green', label='x')
    userData['xy'].plot(userData['timestamps'], userData['y'], c='blue', label='y')
    handles, labels = userData['xy'].get_legend_handles_labels()
    label2handle = dict(zip(labels, handles))
    userData['xy'].legend(label2handle.values(), label2handle.keys())
    userData['longitude_latitude'].set_title('longitude,latitude')
    userData['longitude_latitude'].set_xlabel('time(ms)')
    userData['longitude_latitude'].plot(userData['timestamps'], userData['longitude'], c='green', label='longitude')
    userData['longitude_latitude'].plot(userData['timestamps'], userData['latitude'], c='blue', label='latitude')
    handles, labels = userData['longitude_latitude'].get_legend_handles_labels()
    label2handle = dict(zip(labels, handles))
    userData['longitude_latitude'].legend(label2handle.values(), label2handle.keys())
    plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
