from DataInterfacePython import *
import matplotlib.pyplot as plt

def ModelStart(userData):
    Format = 'time@i,64@[,id@i,direction@b,state@b,timer@i'
    userData['traffic_light'] = BusAccessor(userData['busId'], 'traffic_light', Format)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Bus: traffic_light')

def ModelOutput(userData):
    timestamp, count = userData['traffic_light'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.axes().set_axis_off()
        plt.title('Bus: traffic_light')
        plt.xlim((0, 36))
        plt.ylim((0, 36))
        Direction2Marker = ['v', '<', '^', '>']
        State2Color = ['red', 'yellow', 'green']
        for i in range(count):
            id, direction, state, timer = userData['traffic_light'].readBody(i)
            x, y = ((i%8)+1)*4, 32-(int(i/8))*4
            plt.scatter(x=x, y=y, color=State2Color[state], marker=Direction2Marker[direction])
            plt.text(x=x, y=y+1, s='{}:{}'.format(id, timer), fontsize=10, color=State2Color[state], ha='center', va='center')
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
