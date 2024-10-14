from DataInterfacePython import *
from enum import IntEnum

class Light(IntEnum):
    Width       = 4
    Lower       = 8
    Upper       = 12
    Left        = 16
    Right       = 32
    Emergency   = 48

def ModelStart(userData):
    userData['global_9'] = BusAccessor(userData['busId'], 'global.9', 'time@i,variables@d')
    userData['open_time'] = [(2,  5,  Light.Left),  (6,  9,  Light.Right), (10, 13, Light.Emergency),
                             (14, 17, Light.Width), (18, 21, Light.Lower), (22, 25, Light.Upper)]

def ModelOutput(userData):
    ts = userData['time']
    open = False
    for start, end, light in userData['open_time']:
        variables = (int)(userData['global_9'].readHeader()[1])
        if ts >= start * 1000 and ts < end * 1000:
            variables |= (light.value)
            open = True
            break
    if not open:
        variables &= ~(0x0F << 2)
    userData['global_9'].writeHeader(*(userData['time'], variables))

def ModelTerminate(userData):
    pass
