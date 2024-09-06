from DataInterfacePython import *

def ModelStart(userData):
    Format = 'time@i,valid@b,road_wheel_angle@d'
    userData['wheel_angle'] = BusAccessor(userData['busId'], 'xDriver_road_wheel_angle_input', Format)

def ModelOutput(userData):
    timestamp = userData['time']
    data = [(0, 0), (2000, 1), (4000, 0), (6000, -1), (10000, 0)]
    Valid = 1
    for ts, angle in reversed(data):
        if timestamp >= ts:
            userData['wheel_angle'].writeHeader(*(timestamp, Valid, float(angle)))
            break

def ModelTerminate(userData):
    pass
