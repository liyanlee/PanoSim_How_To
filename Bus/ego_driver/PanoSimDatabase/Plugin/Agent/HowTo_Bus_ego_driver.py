from DataInterfacePython import *
import matplotlib.pyplot as plt

def ModelStart(userData):
    Format = 'time@i,throttle@d,brake@d,steer@d,mode@i,gear@i'
    userData['ego_driver'] = BusAccessor(userData['busId'], 'ego_driver', Format)
    userData['last_time'] = 0
    userData['timestamps'], userData['throttles'], userData['steers'] = [], [], []
    userData['brakes'], userData['modes'], userData['gears'] = [], [], []
    plt.ion()
    fig, (userData['throttle'], userData['steer'], userData['values']) = plt.subplots(1, 3)
    fig.canvas.set_window_title('PanoSim HowTo Bus: ego_driver')
    fig.suptitle('Bus: ego_driver')

def ModelOutput(userData):
    timestamp, throttle, brake, steer, mode, gear = userData['ego_driver'].readHeader()
    if (timestamp - userData['last_time']) >= 100:
        userData['last_time'] = timestamp
        userData['timestamps'].append(timestamp)
        userData['throttles'].append(throttle)
        userData['steers'].append(steer)
        userData['brakes'].append(brake)
        userData['modes'].append(mode)
        userData['gears'].append(gear)
        userData['throttle'].set_title('throttle')
        userData['throttle'].set_xlabel('time(ms)')
        userData['throttle'].plot(userData['timestamps'], userData['throttles'], c='red', label='throttle')
        userData['steer'].set_title('steer(deg)')
        userData['steer'].set_xlabel('time(ms)')
        userData['steer'].plot(userData['timestamps'], userData['steers'], c='red', label='steer')
        userData['values'].set_title('values')
        userData['values'].set_xlabel('time(ms)')
        userData['values'].plot(userData['timestamps'], userData['brakes'], c='red', label='brake')
        userData['values'].plot(userData['timestamps'], userData['modes'], c='green', label='mode')
        userData['values'].plot(userData['timestamps'], userData['gears'], c='blue', label='gear')
        handles, labels = userData['values'].get_legend_handles_labels()
        label2handle = dict(zip(labels, handles))
        userData['values'].legend(label2handle.values(), label2handle.keys())
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
