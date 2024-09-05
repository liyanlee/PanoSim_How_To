from DataInterfacePython import *
import math
from sympy import Matrix, trigsimp, Quaternion
import matplotlib.pyplot as plt

def ModelStart(userData):
    IMU_Format = 'Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Yaw@d,Pitch@d,Roll@d'
    userData['imu'] = BusAccessor(userData['busId'], 'IMU.0', IMU_Format)
    userData['last_imu_data'] = ()
    userData['pose'] = Matrix([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
    userData['velocity'] = Matrix([[0], [0], [0]])
    plt.ion()
    plt.figure(dpi=100).canvas.set_window_title('PanoSim HowTo Sensor: IMU')
    plt.axes().set_aspect('equal')
    plt.title('Sensor: IMU')
    plt.xlabel('X')
    plt.ylabel('Y')

def ModelOutput(userData):
    ts, acc_x, acc_y, _, _, _, gyro_z, _, _, _ = userData['imu'].readHeader()
    if userData['last_imu_data']:
        last_ts, last_acc_x, last_acc_y, last_gyro_z = userData['last_imu_data']
        if ts > last_ts:
            userData['last_imu_data'] = (ts, acc_x, acc_y, gyro_z)
            delta_time = (ts - last_ts) / 1000
            last = Matrix([[0], [0], [last_gyro_z]])
            current = Matrix([[0], [0], [gyro_z]])
            delta_angular = 0.5 * delta_time * (current + last)
            delta_angular_mag = math.sqrt(delta_angular[0]**2 + delta_angular[1]**2 + delta_angular[2]**2)
            delta_angular_dir = delta_angular / delta_angular_mag
            delta_angular_sin = math.sin(delta_angular_mag / 2.0)
            matrix = Matrix([userData['pose'].row(0)[0:3], userData['pose'].row(1)[0:3], userData['pose'].row(2)[0:3]])
            last = matrix * Matrix([last_acc_x, last_acc_y, 0])
            quaternion = Quaternion(math.cos(delta_angular_mag / 2.0), delta_angular_sin * delta_angular_dir[0],
                                    delta_angular_sin * delta_angular_dir[1], delta_angular_sin * delta_angular_dir[2])
            quaternion = trigsimp(Quaternion.from_rotation_matrix(matrix)) * quaternion
            matrix = quaternion.normalize().to_rotation_matrix()
            pose = Matrix([matrix.row(0), matrix.row(1), matrix.row(2), [0,0,0]]).col_insert(3, userData['pose'].col(3))
            current = Matrix([pose.row(0)[0:3], pose.row(1)[0:3], pose.row(2)[0:3]]) * Matrix([acc_x, acc_y, 0])
            delta_velocity = 0.5 * delta_time * (current + last)
            delta = delta_time * userData['velocity'] + 0.5 * delta_time * delta_velocity
            userData['velocity'] += delta_velocity
            pose[0,3] += delta[0,0]
            pose[1,3] += delta[1,0]
            pose[2,3] += delta[2,0]
            userData['pose'] = pose
            plt.scatter(x=pose.col(3)[0,0], y=pose.col(3)[1,0], c='red')
            plt.pause(interval=0.0001)
    else:
        userData['last_imu_data'] = (ts, acc_x, acc_y, gyro_z)

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
