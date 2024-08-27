from DataInterfacePython import *
import matplotlib.pyplot as plt
import numpy as np

def ModelStart(userData):
    sensor_output_format = 'time@i,64@[,OBJ_ID@i,OBJ_Class@i,OBJ_S_Azimuth@d,OBJ_S_Elevation@d,OBJ_S_Velocity@d,OBJ_S_Range@d,OBJ_RCS@d'
    userData['sensor'] = BusAccessor(userData["busId"], 'RadarSensor.0', sensor_output_format)
    userData['last_time'] = 0
    plt.ion()
    userData['figure'] = plt.figure(dpi=100)
    userData['figure'].canvas.set_window_title('PanoSim HowTo Sensor: Radar')
    userData['ax'] = userData['figure'].add_subplot(projection='polar')
    plt.title('Sensor: Radar')
    fov = np.radians(18)
    userData['ax'].set_xlim([-fov/2, fov/2])
    userData['ax'].set_ylim([0, 100])
    userData['ax'].set_theta_direction(-1)
    userData['ax'].set_theta_offset(np.pi/2)
    userData['figure'].canvas.draw()
    userData['0'] = userData['ax'].scatter([], [], c='orange', marker='o', label='Vehicle')
    userData['1'] = userData['ax'].scatter([], [], c='pink', marker='p', label='Pedestrian')
    userData['2'] = userData['ax'].scatter([], [], c='purple', marker='D', label='Other')
    userData['3'] = userData['ax'].scatter([], [], c='cyan', marker='H', label='Obstacle')

def ModelOutput(userData):
    timestamp, count = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        objects = {0:[], 1:[], 2:[], 3:[]}
        for i in range(count):
            _, type, azimuth, _, _, distance, _ = userData['sensor'].readBody(i)
            objects[type].append([azimuth, distance])
        for key in objects.keys():
            if len(objects[key]) > 0:
                userData[str(key)].set_offsets((objects[key]))
            else:
                userData[str(key)].set_offsets([[]])
            userData['ax'].draw_artist(userData[str(key)])
        handles, labels = plt.gca().get_legend_handles_labels()
        label2handle = dict(zip(labels, handles))
        userData['ax'].legend(label2handle.values(), label2handle.keys())
        userData['figure'].canvas.blit(userData['ax'].bbox)
        userData['figure'].canvas.flush_events()

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
