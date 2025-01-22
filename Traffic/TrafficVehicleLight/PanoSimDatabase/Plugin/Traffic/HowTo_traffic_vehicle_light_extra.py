from TrafficModelInterface import *
from BusAccessor import *

def ModelStart(userData):
    userData['id'] = -1
    BusFormat = 'time@i,100@[,id@i,front@b,emergency@b,siren@b'
    userData['light'] = BusAccessor(userData['busId'], 'traffic_vehicle_light_extra', BusFormat)

def ModelOutput(userData):
    id, Speed = userData['id'], 3
    if id > 0:
        front, emergency, siren = 0, 0, 0
        ts = userData['time']
        if ts > 2000 and ts <= 5000:
            front = 1
        elif ts > 6000 and ts <= 9000:
            emergency = 1
        elif ts > 10000 and ts <= 13000:
            siren = 1
        userData['light'].writeBody(0, *(id, front, emergency, siren))
        userData['light'].writeHeader(ts, 1)
        changeSpeed(id, Speed, 0)
    if userData['id'] < 0 and userData['time'] > 1000:
        userData['id'] = addVehicleRelated(0, 3, 0, Speed, lane_type.left, vehicle_type.Van, 101)

def ModelTerminate(userData):
    pass
