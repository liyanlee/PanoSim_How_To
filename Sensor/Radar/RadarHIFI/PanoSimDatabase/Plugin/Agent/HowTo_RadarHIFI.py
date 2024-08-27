from DataInterfacePython import *
import matplotlib.pyplot as plt
import numpy as np

def ModelStart(userData):
    Format = 'time@i,128@[,OBJ_S_Range@d,OBJ_S_Velocity@d,OBJ_S_Azimuth@d,OBJ_S_Elevation@d'
    userData['sensor'] = BusAccessor(userData["busId"], 'RadarHIFISensor.0', Format)
    userData['last_time'] = 0
    plt.ion()
    userData['figure'] = plt.figure(dpi=100)
    userData['figure'].canvas.set_window_title('PanoSim HowTo Sensor: RadarHIFI')
    userData['ax'] = userData['figure'].add_subplot(projection='polar')
    plt.title('Sensor: RadarHIFI')
    fov = np.radians(18)
    userData['ax'].set_xlim([-fov/2, fov/2])
    userData['ax'].set_ylim([0, 100])
    userData['ax'].set_theta_direction(-1)
    userData['ax'].set_theta_offset(np.pi/2)
    userData['figure'].canvas.draw()
    userData['scatter'] = userData['ax'].scatter([], [])

def ModelOutput(userData):
    timestamp, count = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        objects = []
        for i in range(count):
            distance, _, azimuth, _ = userData['sensor'].readBody(i)
            objects.append([azimuth, distance])
        if count > 0:
            userData['scatter'].set_offsets((objects))
        else:
            userData['scatter'].set_offsets([[]])
        userData['ax'].draw_artist(userData['scatter'])
        userData['figure'].canvas.blit(userData['ax'].bbox)
        userData['figure'].canvas.flush_events()

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
