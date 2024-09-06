from DataInterfacePython import *

def ModelStart(userData):
    userData['xDriver_accel_input'] = BusAccessor(userData['busId'], 'xDriver_accel_input', 'time@i,valid@b,accel@d')

def ModelOutput(userData):
    timestamp = userData['time']
    data = [(0, 0), (3000, 5), (6000, 0), (14000, -5), (17000, 0)]
    for ts, accel in reversed(data):
        if timestamp >= ts:
            userData['xDriver_accel_input'].writeHeader(*(timestamp, 1, float(accel)))
            break

def ModelTerminate(userData):
    pass
