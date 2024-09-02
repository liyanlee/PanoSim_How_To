from DataInterfacePython import *
from enum import IntEnum

class WarningType(IntEnum):
    Info    = 0
    Warning = 1
    Error   = 2

def write_warning_bus(userData, warning_type, warning_text):
    bus = userData['warning']
    text_size = len(warning_text)
    max_text_size = bus.maxBodyWidth - 1
    if text_size > max_text_size:
        text_size = max_text_size
        warning_text = warning_text[:max_text_size]
    begin = bus.getHeaderSize()
    end = begin + text_size + 1
    bus.getBus()[begin:end] = '{}\x00'.format(warning_text).encode()
    bus.writeHeader(*(userData['time'], warning_type.value, text_size))

def ModelStart(userData):
    Format = 'time@i,type@b,64@[,text@b'
    userData['warning'] = BusAccessor(userData['busId'], 'warning', Format)
    userData['data'] = [(1000, 2000,  WarningType.Info,    'Info'),
                        (5000, 6000,  WarningType.Warning, 'Warning'),
                        (9000, 10000, WarningType.Error,   'Error')]

def ModelOutput(userData):
    ts = userData['time']
    for start, end, type, text in userData['data']:
        if ts >= start and ts < end:
            write_warning_bus(userData, type, text)

def ModelTerminate(userData):
    pass
