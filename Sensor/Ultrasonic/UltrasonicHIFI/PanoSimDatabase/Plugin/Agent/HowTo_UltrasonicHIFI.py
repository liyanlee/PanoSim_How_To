from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    userData['StartTime'] = 5
    userData['TimeSliceCount'] = 100
    SamplingTimeInterval = 0.25
    userData['StopTime'] = userData['TimeSliceCount'] * SamplingTimeInterval
    Format = 'time@i,mode@i,%d@[,data@d' % userData['TimeSliceCount']
    userData['sensor'] = BusAccessor(userData['busId'], 'UltrasonicHIFISensor.0', Format)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: Ultrasonic HIFI')

def ModelOutput(userData):
    timestamp = userData['sensor'].readHeader()[0]
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        plt.clf()
        plt.title('Sensor: Ultrasonic HIFI')
        plt.xlabel('Time(ms)')
        plt.ylabel('Intensity(dB)')
        plt.ylim((-100, 100))
        data = np.frombuffer(userData['sensor'].getBus()[12:], np.float64)
        intensity = np.log10(data + 1e-30) * 10 if len(data) != 0 else np.zeros((userData['TimeSliceCount']))
        plt.plot(np.linspace(userData['StartTime'], userData['StopTime'], userData['TimeSliceCount']), intensity)
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
