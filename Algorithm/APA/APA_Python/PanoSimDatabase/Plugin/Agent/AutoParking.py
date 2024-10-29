"""
Reference:
#https://zhuanlan.zhihu.com/p/596632865
#https://blog.csdn.net/WaiNgai1999/article/details/132062188
#https://github.com/zhm-real/MotionPlanning/tree/master                   
"""

from matplotlib import pyplot as plt
from DataInterfacePython import *
import time
import math
import numpy as np

import Control.draw as draw
import  Control.Pure_Pursuit as Pure_Pursuit



SPREAD_AXLES = 2.91
STEERING_RATIO = 18
Pure_Pursuit.C.WB = SPREAD_AXLES
RAD_TO_DEG = 180/math.pi

def ModelStart(userData):
    # 构造主车状态总线读取器1
    userData['ego_state'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d') 
    # 主车可泊入车位信息
    userData["spot"] = BusAccessor(userData["busId"], "ego_parking_spot", "time@i,valid@b,x1@d,y1@d,x2@d,y2@d,x3@d,y3@d,x4@d,y4@d")
    
    userData["ego_throttle"] = BusAccessor(userData["busId"], "ego_control.throttle", "time@i,valid@b,throttle@d")
    userData["ego_brake"] = BusAccessor(userData["busId"], "ego_control.brake", "time@i,valid@b,brake@d")
    userData["ego_steer"] = BusAccessor(userData["busId"], "ego_control.steer", "time@i,valid@b,steer@d")
    userData["ego_mode"] = BusAccessor(userData["busId"], "ego_control.mode", "time@i,valid@b,mode@i")

    userData["generated_path"]  = None
    userData["x_rec"] = []
    userData["y_rec"] = []
    userData["pursuit_time"] = 0.0
    userData["pursuit_index"] = 0

def ModelOutput(userData):
    ego_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = userData['ego_state'].readHeader()
    ego_x = ego_x - SPREAD_AXLES *math.cos(ego_yaw) 
    ego_y = ego_y - SPREAD_AXLES*math.sin(ego_yaw) 
    if ego_yaw < 0:
        ego_yaw += 2*math.pi

    spot_time,spot_valid,x1,y1,x2,y2,x3,y3,x4,y4 = userData['spot'].readHeader()
    if userData["generated_path"] is None and spot_time > 0 and spot_valid == 1:
        start_point = (ego_x, ego_y, ego_yaw*RAD_TO_DEG)
        end_point = ((x1+x2+x3+x4)/4, (y1+y2+y3+y4)/4 + (y1-y3)*(1/6), math.degrees(math.atan2((y2-y1), (x2-x1))))
        userData["generated_path"] = Pure_Pursuit.generate_path([start_point,end_point])

    if userData["generated_path"] and userData["pursuit_index"] < len(userData["generated_path"][0]):
        path_x,path_y,path_yaw,direct, x_all, y_all = userData["generated_path"]
        i = userData["pursuit_index"]
        Pure_Pursuit.C.WB = SPREAD_AXLES
        node = Pure_Pursuit.Node(x=ego_x, y=ego_y, yaw=ego_yaw, v=ego_speed, direct=direct[0][0])
        nodes = Pure_Pursuit.Nodes()
        nodes.add(userData["pursuit_time"], node)
        ref_trajectory = Pure_Pursuit.PATH(path_x[i], path_y[i])
        target_ind, _ = ref_trajectory.target_index(node)
        if direct[i][0] > 0:
            target_speed = 3.0 / 3.6
            Pure_Pursuit.C.Ld = 1.0  #4.0
            Pure_Pursuit.C.dist_stop = 0.25 #1.5
            Pure_Pursuit.C.dc = -1.1
        else:
            target_speed = 3.0 / 3.6
            Pure_Pursuit.C.Ld = 0.5  #2.5
            Pure_Pursuit.C.dist_stop = 0.1 #0.2
            Pure_Pursuit.C.dc = 0.2
        dist = math.hypot(ego_x - path_x[i][target_ind], ego_y - path_y[i][target_ind])
        if dist < Pure_Pursuit.C.dist_stop:
            userData["pursuit_time"] = 0.0
            userData["pursuit_index"] += 1
        acceleration = Pure_Pursuit.pid_control(target_speed, node.v, dist, direct[i][0])
        delta, target_ind = Pure_Pursuit.pure_pursuit(node, ref_trajectory, target_ind)
        userData["pursuit_time"] += Pure_Pursuit.C.dt
        L = Pure_Pursuit.C.WB
        ld = math.hypot(ego_x - path_x[i][target_ind], ego_y - path_y[i][target_ind])
        P = 2*L/ld/ld
        
        ey = P*delta
        K = STEERING_RATIO 
        
        steer = K*math.atan(2.0 * L * ey / (ld*ld))*RAD_TO_DEG/4
        userData["x_rec"].append(node.x)
        userData["y_rec"].append(node.y)
        throttle = 0.00 # acceleration
        if ego_speed < 0.1:
            throttle = 0.04
        brake = 0.68    # brake
        mode = direct[i][0]  # 1 for D and -1 for R
        if i == len(path_x)-1 and dist < Pure_Pursuit.C.dist_stop:  # stop while finish parking
            brake = 15
            steer = 0 
            mode = 0
        userData["ego_throttle"].writeHeader(*(ego_time, 1, throttle))
        userData["ego_brake"].writeHeader(*(ego_time, 1, brake))
        userData["ego_steer"].writeHeader(*(ego_time, 1, steer))
        userData["ego_mode"].writeHeader(*(ego_time, 1, mode))

        # plt for gui
        plt.cla()
        plt.plot(node.x, node.y, marker='.', color='k')
        plt.plot(x_all , y_all , color='gray', linewidth=2)
        plt.plot(userData["x_rec"], userData["y_rec"], color='darkviolet', linewidth=2)
        plt.plot(path_x[i][target_ind], path_y[i][target_ind], ".r")
        draw.draw_car(node.x, node.y, ego_yaw, -steer/STEERING_RATIO/RAD_TO_DEG , Pure_Pursuit.C)

        plt.axis("equal")
        plt.title("AutoParking: v=" + str(node.v * 3.6)[:4] + "km/h")
        plt.gcf().canvas.mpl_connect('key_release_event',
                                    lambda event:
                                    [exit(0) if event.key == 'escape' else None])
        plt.pause(0.001)
            
    
 
def ModelTerminate(userData):
    plt.close()
