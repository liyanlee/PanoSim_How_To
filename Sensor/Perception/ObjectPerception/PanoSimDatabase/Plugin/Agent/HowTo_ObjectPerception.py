from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    OutputFormat = 'Timestamp@i,64@[,OBJ_ID@i,OBJ_Class@b,\
        OBJ_X@d,OBJ_Y@d,OBJ_Z@d,OBJ_Velocity@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d'
    userData['sensor'] = BusAccessor(userData['busId'], 'ObjectPerception.0', OutputFormat)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Object Perception')

def ModelOutput(userData):
    timestamp, count = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp

        plt.clf()
        axes = plt.axes()
        axes.set_aspect('equal')
        plt.title('Sensor: Object Perception')
        plt.ylabel('X')
        plt.ylim((-10, 110))
        plt.yticks(np.arange(0, 110, 10))
        plt.xlabel('Y')
        plt.xlim((-60, 60))
        xTicks = np.arange(-50, 60, 10)
        plt.xticks(xTicks)
        axes.set_xticklabels(['{}'.format(i) for i in xTicks[::-1]])

        Type2Style = [('Vehicle', 'orange', 'o'), ('Pedestrian', 'pink', 'p'),
                      ('Other', 'purple', 'D'), ('Sign', 'blue', 'h'), ('Obstacle', 'cyan', 'H')]
        plt.scatter(x=0, y=0, color='red', marker='s', label='ego')
        for i in range(count):
            _, type, x, y, _, _, _, _, _ = userData['sensor'].readBody(i)
            label, color, marker = Type2Style[type]
            plt.scatter(x=-y, y=x, c=color, marker=marker, label=label)

        handles, labels = plt.gca().get_legend_handles_labels()
        label2handle = dict(zip(labels, handles))
        plt.legend(label2handle.values(), label2handle.keys())
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
