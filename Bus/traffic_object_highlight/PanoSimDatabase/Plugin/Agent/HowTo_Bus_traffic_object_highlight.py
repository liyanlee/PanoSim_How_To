from DataInterfacePython import *

def ModelStart(userData):
    Format = 'time@i,100@[,id@i,r@b,g@b,b@b,a@b'
    userData['bus'] = BusAccessor(userData['busId'], 'traffic_object_highlight', Format)

def ModelOutput(userData):
    id, red, green, blue, alpha = 109, 127, 0, 0, 127
    if userData['time'] > 2000:
        userData['bus'].writeHeader(*(userData['time'], 1))
        userData['bus'].writeBody(0, *(id, red, green, blue, alpha))

def ModelTerminate(userData):
    pass
