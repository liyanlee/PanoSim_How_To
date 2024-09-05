from DataInterfacePython import *

def ModelStart(userData):
    userData['ego_control.brake'] = BusAccessor(userData['busId'], 'ego_control.brake', 'time@i,valid@b,brake@d')
    userData['ego_control.throttle'] = BusAccessor(userData['busId'], 'ego_control.throttle', 'time@i,valid@b,throttle@d')

def ModelOutput(userData):
    timestamp = userData['time']
    valid = 1 if timestamp >= 3000 and timestamp < 6000 else 0
    userData['ego_control.brake'].writeHeader(*(timestamp, valid, float(5)))
    userData['ego_control.throttle'].writeHeader(*(timestamp, valid, float(0)))

def ModelTerminate(userData):
    pass
