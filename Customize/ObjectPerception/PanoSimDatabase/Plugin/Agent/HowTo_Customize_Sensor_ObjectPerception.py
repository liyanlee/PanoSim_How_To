from DataInterfacePython import *
import numpy as np
from shapely import geometry
from shapely.strtree import STRtree
import matplotlib.pyplot as plt

def generate_sector(distance, fov, yaw_install, ego_x, ego_y, ego_yaw):
    perimeter = 2 * np.pi * distance
    arc_length = perimeter * fov / 360.0
    number_segments = int(arc_length // 2) + 1
    start_angle = ego_yaw - np.radians(fov) / 2 - yaw_install
    end_angle = ego_yaw + np.radians(fov) / 2 - yaw_install
    theta = np.linspace(start_angle, end_angle, number_segments)
    x = ego_x + distance * np.cos(theta)
    y = ego_y + distance * np.sin(theta)
    points_array = np.column_stack([x, y])
    if fov < 360:
        points_array = np.vstack((points_array, np.array([ego_x, ego_y])))
    return geometry.Polygon(points_array)

def calc_range_bbox(start_x, start_y, end_x, end_y, vertexs):
    x1, y1, _ = vertexs[1]
    x2, y2, _ = vertexs[2]
    x3, y3, _ = vertexs[3]
    x4, y4, _ = vertexs[4]
    linear_ring = geometry.LinearRing([(x1, y1), (x2, y2), (x4, y4), (x3, y3)])
    ray = geometry.LineString([(start_x, start_y), (end_x, end_y)])
    intersection_ = linear_ring.intersection(ray)
    range_bbox = 0.0
    if not intersection_.is_empty:
        if not intersection_.geom_type.startswith('Multi') and intersection_.geom_type != 'GeometryCollection':
            first_point = geometry.Point(intersection_.coords[0])
            range_bbox = geometry.Point(start_x, start_y).distance(geometry.Point(first_point.x, first_point.y))
    return range_bbox

def check_occlusion(tree, query_geom, type):
    intersection_count = 0
    for item in tree.query(query_geom):
        if item.intersects(query_geom):
            if type != 0:
                return True
            else:
                intersection_count += 1
                if intersection_count > 1:
                    return True
    return False

def calculate_occlusion(output, timestamp, objects, ego_x, ego_y, ego_z, ego_yaw, yaw_install):
    linear_rings = []
    for object in objects:
        _, type, _, _, _, _, _, _, vertexs = object
        if type == 0:
            x1, y1, _ = vertexs[1]
            x2, y2, _ = vertexs[2]
            x3, y3, _ = vertexs[3]
            x4, y4, _ = vertexs[4]
            linear_ring = geometry.LinearRing([(x1, y1), (x2, y2), (x4, y4), (x3, y3)])
            linear_rings.append(linear_ring)
    tree = STRtree(linear_rings)
    index = 0
    for object in objects:
        id, type, shape, x, y, z, yaw, speed, vertexs = object
        if check_occlusion(tree, geometry.LineString([(ego_x, ego_y), (x, y)]), type):
            continue
        range_center = geometry.Point(ego_x, ego_y).distance(geometry.Point(x, y))
        range_bbox = calc_range_bbox(ego_x, ego_y, x, y, vertexs)
        azimuth_angle = -((np.arctan2(y - ego_y, x - ego_x) + yaw_install - ego_yaw + np.pi) % (np.pi * 2) - np.pi)
        elevation_angle = (np.arctan2(z - ego_z, geometry.Point(ego_x, ego_y).distance(geometry.Point(x, y)))) % (np.pi / 2)
        heading = np.radians(yaw) % (2 * np.pi)
        output.writeBody(index, *(id, type, shape, range_center, range_bbox, azimuth_angle, elevation_angle, speed, heading))
        index += 1
    output.writeHeader(*(timestamp, index))

def ModelStart(userData):
    userData['distance'] = float(100)
    userData['fov'] = float(60)
    userData['yaw'] = float(0)
    userData['occlusion'] = False

    bus_id = userData['busId']
    OutputFormat = 'time@i,100@[,id@i,type@b,shape@i,range_center@f,range_bbox@f,azimuth_angle@f,elevation_angle@f,velocity@f,heading@f'
    userData['output'] = BusAccessor(bus_id, 'Customize_Sensor_ObjectPerception', OutputFormat)
    userData['ego'] = BusAccessor(bus_id, 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    userData['traffic'] = DoubleBusReader(bus_id, 'traffic', 'time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f')

    distance = float(userData['distance'])
    fov = float(userData['fov'])
    yaw_install = float(userData['yaw'])
    plt.ion()
    userData['figure'] = plt.figure('Customize_Sensor_ObjectPerception')
    userData['ax'] = userData['figure'].add_subplot(projection='polar')
    if fov < 360:
        fov = np.radians(fov)
        userData['ax'].set_xlim([-fov/2, fov/2])
    userData['ax'].set_ylim([0, distance])
    userData['ax'].set_theta_direction(-1)
    userData['ax'].set_theta_offset(np.pi/2 - np.radians(yaw_install))
    userData['figure'].canvas.draw()
    userData['bg'] = userData['figure'].canvas.copy_from_bbox(userData['ax'].bbox)
    userData['0'] = userData['ax'].scatter([], [], c='b', marker='s', label='vehicle')
    userData['1'] = userData['ax'].scatter([], [], c='g', marker='^', label='pedestrian')
    userData['2'] = userData['ax'].scatter([], [], c='r', marker='d', label='other')

def ModelOutput(userData):
    distance = float(userData['distance'])
    fov = float(userData['fov'])
    yaw_install = np.radians(float(userData['yaw']))
    calc_occlusion = userData['occlusion'] == 'TRUE'
    output = userData['output']
    timestamp = userData['time']

    _, ego_x, ego_y, ego_z, ego_yaw, _, _, _ = userData['ego'].readHeader()
    sector_polygon = generate_sector(distance, fov, yaw_install, ego_x, ego_y, ego_yaw)
    min_x, min_y, max_x, max_y = sector_polygon.bounds
    min_x -= 10
    min_y -= 10
    max_x += 10
    max_y += 10

    trafffic = userData['traffic'].getReader(timestamp)
    _, width = trafffic.readHeader()
    index = 0
    objects = []
    for i in range(width):
        id, type, shape, x, y, z, yaw, pitch, roll, speed = trafffic.readBody(i)
        if x > min_x and y > min_y and x < max_x and y < max_y:
            vertexs = getObjectVertex(object_type(type), shape, x, y, z, yaw, pitch, roll)
            if type == 0:
                x, y, z = vertexs[0]
            if sector_polygon.contains(geometry.Point(x, y)):
                if calc_occlusion:
                    objects.append((id, type, shape, x, y, z, yaw, speed, vertexs))
                else:
                    range_center = geometry.Point(ego_x, ego_y).distance(geometry.Point(x, y))
                    range_bbox = calc_range_bbox(ego_x, ego_y, x, y, vertexs)
                    azimuth_angle = -((np.arctan2(y - ego_y, x - ego_x) + yaw_install - ego_yaw + np.pi) % (np.pi * 2) - np.pi)
                    elevation_angle = (np.arctan2(z - ego_z, geometry.Point(ego_x, ego_y).distance(geometry.Point(x, y)))) % (np.pi / 2)
                    heading = np.radians(yaw) % (2 * np.pi)
                    output.writeBody(index, *(id, type, shape, range_center, range_bbox, azimuth_angle, elevation_angle, speed, heading))
                    index += 1

    if calc_occlusion:
        calculate_occlusion(output, timestamp, objects, ego_x, ego_y, ego_z, ego_yaw, yaw_install)
    else:
        output.writeHeader(*(timestamp, index))

    userData['figure'].canvas.restore_region(userData['bg'])
    objects = {0:[], 1:[], 2:[]}
    _, width = output.readHeader()
    for index in range(width):
        _, type, _, _, range_bbox, azimuth_angle, _, _, _ = output.readBody(index)
        objects[type].append([azimuth_angle, range_bbox])
    for key in objects.keys():
        if len(objects[key]) > 0:
            userData[str(key)].set_offsets((objects[key]))
        else:
            userData[str(key)].set_offsets([[]])
        userData['ax'].draw_artist(userData[str(key)])
    handles, labels = plt.gca().get_legend_handles_labels()
    label2handle = dict(zip(labels, handles))
    plt.legend(label2handle.values(), label2handle.keys())
    userData['figure'].canvas.blit(userData['ax'].bbox)
    userData['figure'].canvas.flush_events()

def ModelTerminate(userData):
    plt.close()
