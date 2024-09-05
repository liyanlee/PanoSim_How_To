from DataInterfacePython import *
import matplotlib.pyplot as plt

def ModelStart(userData):
    OutputFormat = 'Timestamp@i,4@[,TrafficLight_Direction@b,TrafficLight_State@b,TrafficLight_Timer@i'
    userData['sensor'] = BusAccessor(userData['busId'], 'TrafficLightPerception.0', OutputFormat)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Traffic Light Perception')

def ModelOutput(userData):
    timestamp, count = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.axes().set_axis_off()
        plt.title('Sensor: Traffic Light Perception')
        plt.xlim((-10, 10))
        plt.ylim((-10, 10))
        Direction2Marker = ['v', '<', '^', '>']
        State2Color = ['red', 'yellow', 'green']
        Count2Offset = [0, 0, -1, -2, -3]
        x_offset = Count2Offset[count]
        for i in range(count):
            direction, state, timer = userData['sensor'].readBody(i)
            plt.scatter(x=x_offset, y=0, color=State2Color[state], marker=Direction2Marker[direction])
            plt.text(x=x_offset, y=1, s=str(timer), fontsize=10, color=State2Color[state], ha='center', va='center')
            x_offset += 2
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
