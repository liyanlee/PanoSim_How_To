from DataInterfacePython import *
import matplotlib.pyplot as plt

def ModelStart(userData):
    bus_ego_format = 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d'
    userData['ego'] = BusAccessor(userData['busId'], 'ego', bus_ego_format)
    Format = 'time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f'
    userData['traffic'] = DoubleBusReader(userData['busId'], 'traffic', Format)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Bus: traffic')

def ModelOutput(userData):
    timestamp, ego_x, ego_y, _, _, _, _, _ = userData['ego'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.axes().set_aspect('equal')
        plt.title('Bus: traffic')
        plt.xlabel('X')
        plt.ylabel('Y')
        Type2Style = [('Vehicle', 'orange', 'o'), ('Pedestrian', 'pink', 'p'), ('Other', 'purple', 'D')]
        trafffic_bus = userData['traffic'].getReader(userData['time'])
        _, width = trafffic_bus.readHeader()
        for i in range(width):
            _, type, _, x, y, _, _, _, _, _ = trafffic_bus.readBody(i)
            label, color, marker = Type2Style[type]
            plt.scatter(x=x, y=y, c=color, marker=marker, label=label)
        plt.scatter(x=ego_x, y=ego_y, c='r', marker='s', label='ego')
        handles, labels = plt.gca().get_legend_handles_labels()
        label2handle = dict(zip(labels, handles))
        plt.legend(label2handle.values(), label2handle.keys())
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
