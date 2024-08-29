from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt

def ModelStart(userData):
    bus_ego_format = 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d'
    userData['ego'] = BusAccessor(userData['busId'], 'ego', bus_ego_format)
    userData['last_time'] = 0
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Bus: ego')
    userData['ax'] = plt.axes(projection='3d')

def ModelOutput(userData):
    timestamp, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, _ = userData['ego'].readHeader()
    if timestamp > userData['last_time']:
        userData['last_time'] = timestamp
        Length, Width, Height, FrontOverhang = 4.98, 2.06, 1.42, 0.96
        dx, dy, dz = Length/2, Width/2, Height/2
        points_x, points_y, points_z = np.meshgrid([-dx-FrontOverhang, dx-FrontOverhang], [-dy, dy], [-dz, dz])
        pts = np.vstack([points_x.ravel(), points_y.ravel(), points_z.ravel()]).T
        rotate_x = np.array([[1, 0, 0, 0], [0, np.cos(ego_roll), -np.sin(ego_roll), 0],
                    [0, np.sin(ego_roll), np.cos(ego_roll), 0], [0, 0, 0, 1]])
        rotate_y = np.array([[np.cos(ego_pitch), 0, np.sin(ego_pitch), 0], [0, 1, 0, 0],
                    [-np.sin(ego_pitch), 0, np.cos(ego_pitch), 0], [0, 0, 0, 1]])
        rotate_z = np.array([[np.cos(ego_yaw), -np.sin(ego_yaw), 0, 0],
                    [np.sin(ego_yaw), np.cos(ego_yaw), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rotate = rotate_y @ rotate_x @ rotate_z
        points = np.hstack((pts, np.ones((len(pts), 1))))
        rotated_points = np.dot(points, rotate.T)
        pts = rotated_points[:, :3]
        pts += (ego_x, ego_y, ego_z)
        LinePoints = [(0,1), (2,3), (4,5), (6,7), (0,2), (0,4), (2,6), (4,6), (1,3), (1,5), (3,7), (5,7)]
        plt.cla()
        plt.title('Bus: ego')
        userData['ax'].set_xlabel('X')
        userData['ax'].set_ylabel('Y')
        userData['ax'].set_zlabel('Z')
        userData['ax'].set_xlim((ego_x-5, ego_x+5))
        userData['ax'].set_ylim((ego_y-5, ego_y+5))
        userData['ax'].set_zlim((ego_z-5, ego_z+5))
        userData['ax'].scatter(ego_x, ego_y, ego_z, c='red')
        for i, j in LinePoints:
            userData['ax'].plot(
                xs=[pts[i][0], pts[j][0]], ys=[pts[i][1], pts[j][1]], zs=[pts[i][2], pts[j][2]], c='red')
        plt.pause(interval=0.0001)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
