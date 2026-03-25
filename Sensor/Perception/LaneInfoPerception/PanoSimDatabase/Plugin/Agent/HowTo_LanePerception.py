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
        Slot2Flag = ['LL', 'L', 'R', 'RR']
        Id2Flag = {0: 'LL', 1: 'L', 2: 'R', 3: 'RR'}
        # Lane type definition:
        # 0 None, 1 SingleWhite, 2 SingleYellow, 3 BrokenWhite,
        # 4 BrokenYellow, 5 DoubleWhite, 6 DoubleYellow, 7 Unknown.
        Type2ColorStyle = {
            1: ('w', '-'),
            2: ('y', '-'),
            3: ('w', ':'),
            4: ('y', ':'),
            5: ('w', '-.'),
            6: ('y', '-.'),
            7: ('c', '--')
        }
        
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
            lane_id, _, left, right, curvature, c0, c1, c2, c3, lane_type = userData['sensor_output'].readBody(i)
            

            # Use sensor lane type semantics directly: 0 means no lane.
            lane_type = int(lane_type)
            lane_id = int(lane_id)

            all_zero_lane = (
                left == 0 and right == 0 and curvature == 0 and
                c0 == 0 and c1 == 0 and c2 == 0 and c3 == 0
            )
            if lane_type == 0 or all_zero_lane:
                continue

            if lane_id in Id2Flag:
                flag = Id2Flag[lane_id]
            elif 0 <= i < len(Slot2Flag):
                flag = Slot2Flag[i]
            else:
                flag = 'ID{}'.format(lane_id)

            # The 4 output slots are fixed as LL/L/R/RR. Use L/R slot for distance display.
            if flag == 'L':
                distance2line = (left, right)
            elif flag == 'R' and not distance2line:
                distance2line = (left, right)
            points_x = []
            points_y = []
            for k in range(1000):
                x = k * 0.1
                y = c0 + c1 * x + c2 * x * x + c3 * x * x * x
                points_x.append(np.cos(Half_PI) * x - np.sin(Half_PI) * y)
                points_y.append(np.sin(Half_PI) * x + np.cos(Half_PI) * y)
            curvatures.append((points_x[120] - 1, points_y[120], flag, '{:.2f}'.format(curvature)))

            color, style = Type2ColorStyle.get(lane_type, ('c', '-'))
            plt.plot(points_x, points_y, color=color, linestyle=style, linewidth=2.0)
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
        plt.draw()
        plt.pause(interval=0.001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
