from DataInterfacePython import *
from shapely.geometry import Polygon

def check_collision(userData):
    _, ego_x, ego_y, ego_z, _, _, _, _ = userData['ego'].readHeader()
    ego = getEgoVertex()
    ego_polygon = Polygon([(ego[1][0], ego[1][1]), (ego[2][0], ego[2][1]), (ego[3][0], ego[3][1]), (ego[4][0], ego[4][1])])
    bus_traffic = userData['bus_traffic'].getReader(userData['time'])
    _, width = bus_traffic.readHeader()
    for index in range(width):
        _, type, shape, x, y, z, yaw, pitch, roll, _ = bus_traffic.readBody(index)
        if abs(ego_x - x) > 20 or abs(ego_y - y) > 20 or abs(ego_z - z) > 1:
            continue
        pts = getObjectVertex(object_type(type), shape, x, y, z, yaw, pitch, roll)
        object = Polygon([(pts[1][0], pts[1][1]), (pts[2][0], pts[2][1]), (pts[3][0], pts[3][1]), (pts[4][0], pts[4][1])])
        if ego_polygon.intersects(object):
            return True
    return False

def ModelStart(userData):
    userData['ego'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    traffic_format = 'time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f'
    userData['bus_traffic'] = DoubleBusReader(userData['busId'], 'traffic', traffic_format)
    userData['judge'] = BusAccessor(userData['busId'], 'judge.0', 'time@i,judge@d')

def ModelOutput(userData):
    collision_event = 1 if check_collision(userData) else 0
    userData['judge'].writeHeader(*(userData['time'], collision_event))

def ModelTerminate(userData):
    pass
