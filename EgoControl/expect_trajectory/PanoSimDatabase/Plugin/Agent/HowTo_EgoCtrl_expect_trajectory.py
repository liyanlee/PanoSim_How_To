from DataInterfacePython import *
from shapely import geometry, ops, affinity

def ModelStart(userData):
    userData['ego'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    userData['xDriver_path_input'] = BusAccessor(userData['busId'], 'xDriver_path_input', 'time@i,valid@b,500@[,x@d,y@d')
    userData['trajectory'] = geometry.LineString(getWayPoints())
    userData['start'] = 0

def ModelOutput(userData):
    _, ego_x, ego_y, _, ego_yaw, _, _, _ = userData['ego'].readHeader()
    length = 0
    if userData['start'] < userData['trajectory'].length:
        offset = ops.substring(userData['trajectory'], userData['start'], userData['start'] + 20).project(geometry.Point(ego_x, ego_y))
        userData['start'] += offset
        path = ops.substring(userData['trajectory'], userData['start'], userData['start'] + 100)
        path = affinity.translate(path, -ego_x, -ego_y)
        path = affinity.rotate(path, -ego_yaw, origin=(0, 0), use_radians=True)
        coords = path.coords
        length = len(coords)
        for i in range(length):
            userData['xDriver_path_input'].writeBody(i, *(coords[i][0], coords[i][1]))
    userData['xDriver_path_input'].writeHeader(*(userData['time'], 1, length))

def ModelTerminate(userData):
    pass
