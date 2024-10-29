from DataInterfacePython import *
import numpy as np
from shapely.geometry import Polygon

def parkingSpotWarning(userData):
    if "warning" not in userData:
        userData["warning"] = True
        bus = BusAccessor(userData['busId'], "warning", "time@i,type@b,64@[,text@b")
        text = "Only Support Vertical Parking"
        body = bus.getHeaderSize()
        bus.getBus()[body:body + len(text)] = text.encode()
        bus.writeHeader(*(userData["time"], 0, len(text)))

def ModelStart(userData):
    userData["ego"] = BusAccessor(userData['busId'], "ego", EGO_FORMAT)
    userData["traffic"] = DoubleBusReader(userData["busId"], "traffic", TRAFFIC_FORMAT)
    userData["spot"] = BusAccessor(userData["busId"], "ego_parking_spot", "time@i,valid@b,x1@d,y1@d,x2@d,y2@d,x3@d,y3@d,x4@d,y4@d")
    userData["start"] = False
    userData["id"] = getTargetParkingSpot()

def ModelOutput(userData):
    if userData["time"] < 100:
        return
    
    ego = userData["ego"].readHeader()
    if not userData["start"]:
        spots = getParkingSpotsV2(9)
        if len(spots) > 0:
            other = []
            for obstacle in getObstacleV2(15):
                if np.abs(obstacle[5] - ego[3]) > 1:
                    continue
                vertex = getObjectVertex(*obstacle[1:])
                other.append(Polygon(v[0:2] for v in vertex[1:5]))
            traffic = userData["traffic"].getReader(userData["time"])
            _, width = traffic.readHeader()
            for i in range(width):
                item = traffic.readBody(i)
                if np.abs(item[5] - ego[3]) > 1:
                    continue
                if np.linalg.norm(np.array((ego[1],ego[2])) - np.array((item[3], item[4]))) < 15:
                    vertex = getObjectVertex(object_type(item[1]), *item[2:9])
                    other.append(Polygon(v[0:2] for v in vertex[1:5]))
        for spot in spots:
            if userData["id"] > 0:
                if spot[0] != userData["id"]:
                    continue
            angle = np.arctan2(spot[4] - ego[2], spot[3] - ego[1]) - ego[4]
            angle = angle % (np.pi * 2)
            if angle < np.pi * 8/6 or angle > np.pi * 10/6:
                # only right spots to ego
                continue
            angle = np.arctan2(spot[6] - spot[4], spot[5] - spot[3]) - ego[4]
            angle = angle % (np.pi * 2)
            if angle < np.pi * 5/12 or angle > np.pi * 7/12:
                # only vertical spots to ego
                parkingSpotWarning(userData)
                continue
            angle = np.arctan2(spot[6] - spot[4], spot[5] - spot[3]) - np.arctan2(spot[8] - spot[6], spot[7] - spot[5])
            angle = angle % (np.pi * 2)
            if angle < np.pi * 5/12 or angle > np.pi * 7/12:
                # only vertical spots
                parkingSpotWarning(userData)
                continue
            target = Polygon([spot[i:i+2] for i in range(3,11,2)])
            occupied = False
            for o in other:
                if target.intersects(o):
                    occupied = True
                    break
            if not occupied:
                userData["spot"].writeHeader(*(userData["time"], 1, *spot[3:]))
                userData["start"] = True
                break

def ModelTerminate(userData):
    pass