from DataInterfacePython import *
import matplotlib.pyplot as plt

def ModelStart(userData):
    OutputFormat = 'time@i,64@[,x1@d,y1@d,x2@d,y2@d,x3@d,y3@d,x4@d,y4@d'
    userData['sensor'] = BusAccessor(userData["busId"], 'ParkingLotsPerception.0', OutputFormat)
    EgoFormat = 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d'
    userData['ego'] = BusAccessor(userData['busId'], 'ego', EgoFormat)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Parking Lots Perception')

def ModelOutput(userData):
    timestamp, count = userData['sensor'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.title('Sensor: Parking Lots Perception')
        ego = userData['ego'].readHeader()
        plt.scatter(x=ego[1], y=ego[2], color='red', marker='s', label='ego')
        for i in range(count):
            x1, y1, x2, y2, x3, y3, x4, y4 = userData['sensor'].readBody(i)
            x = [x1, x2, x3, x4, x1]
            y = [y1, y2, y3, y4, y1]
            plt.plot(x, y, color='blue', label='Parking Lots')
        handles, labels = plt.gca().get_legend_handles_labels()
        label2handle = dict(zip(labels, handles))
        plt.legend(label2handle.values(), label2handle.keys())
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
