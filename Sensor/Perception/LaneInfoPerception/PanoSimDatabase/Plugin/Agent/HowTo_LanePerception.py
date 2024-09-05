from DataInterfacePython import *
import matplotlib.pyplot as plt
import numpy as np

def ModelStart(userData):
    sensor_name = 'LaneInfoPerception.0'
    sensor_output_format = 'Timestamp@i,4@[,Lane_ID@i,Lane_Distance@d,\
        Lane_Car_Distance_Left@d,Lane_Car_Distance_Right@d,Lane_Curvature@d,\
        Lane_Coefficient_C0@d,Lane_Coefficient_C1@d,Lane_Coefficient_C2@d,Lane_Coefficient_C3@d,Lane_Class@b'
    userData['sensor_output'] = BusAccessor(userData['busId'], sensor_name, sensor_output_format)
    userData['last'] = 0
    plt.ion()
    plt.rcParams['figure.figsize'] = [5, 7]
    figure = plt.figure(dpi=100)
    figure.canvas.set_window_title('PanoSim HowTo Sensor: Lane Info Perception')

def ModelOutput(userData):
    if (userData['time'] - userData['last']) >= 100:
        userData['last'] = userData['time']
        Half_PI = np.pi * 0.5
        Id2Flag = ['LL', 'L', 'R', 'RR']
        Type2ColorStyle = [('', ''), ('w', '-'), ('y', '-'), ('w', ':'), ('y', ':'), ('w', '-.'), ('y', '-.')]
        
        plt.clf()
        axes = plt.axes()
        axes.set_facecolor('#B39681')
        axes.set_aspect('equal')
        plt.title('Sensor: Lane Info Perception')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.xlim((-30, 30))
        plt.ylim((-5, 105))

        curvatures = []
        distance2line = ()
        _, width = userData['sensor_output'].readHeader()
        for i in range(width):
            id, _, left, right, curvature, c0, c1, c2, c3, type = userData['sensor_output'].readBody(i)
            if type != 0:
                if not distance2line:
                    distance2line = (left, right)
                points_x = []
                points_y = []
                for k in range(1000):
                    x = k * 0.1
                    y = c0 + c1 * x + c2 * x * x + c3 * x * x * x
                    points_x.append(np.cos(Half_PI) * x - np.sin(Half_PI) * y)
                    points_y.append(np.sin(Half_PI) * x + np.cos(Half_PI) * y)
                curvatures.append((points_x[120] - 1, points_y[120], Id2Flag[id], '{:.2f}'.format(curvature)))
                color, style = Type2ColorStyle[type]
                plt.plot(points_x, points_y, color=color, linestyle=style)
        if distance2line:
            left, right = distance2line
            plt.text(x=-28, y=99, s='distance to left line: {:.2f}'.format(left), color='blue')
            plt.text(x=-28, y=96, s='distance to right line: {:.2f}'.format(right), color='blue')
        y_offset = 0
        for (x, y, flag, curvature) in curvatures:
            plt.text(x=x, y=y, s=flag, color='blue')
            plt.text(x=-28, y=(93 - y_offset), s='{} curvature: {}'.format(flag, curvature), color='blue')
            y_offset += 3
        plt.scatter(x=0, y=0, color='red', marker='s')
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
