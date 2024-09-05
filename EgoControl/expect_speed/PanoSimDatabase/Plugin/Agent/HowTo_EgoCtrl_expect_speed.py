from DataInterfacePython import *

def ModelStart(userData):
    userData["xDriver_speed_input"] = BusAccessor(userData["busId"], "xDriver_speed_input", "time@i,valid@b,speed@d,accel@d")

def ModelOutput(userData):
    timestamp = userData['time']
    valid = 1 if timestamp >= 5000 and timestamp < 10000 else 0
    Speed, Acceleration = 10, 0
    userData["xDriver_speed_input"].writeHeader(*(timestamp, valid, float(Speed), float(Acceleration)))

def ModelTerminate(userData):
    pass
