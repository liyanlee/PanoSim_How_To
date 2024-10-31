from DataInterfacePython import *
import math
 
def Warning(userData,level,warn):
    bus = userData["warning"].getBus()
    size = userData["warning"].getHeaderSize()
    bus[size:size + len(warn)] = '{}'.format(warn).encode()
    userData["warning"].writeHeader(*(userData["time"], level, len(warn)))

# 仿真实验启动时回调
def ModelStart(userData):
    # 短接部分控制信号 油门
    userData['ego_throttle'] = BusAccessor(userData['busId'],'ego_control.throttle','time@i,valid@b,throttle@d') 
    # 短接部分控制信号 刹车
    userData['ego_brake'] = BusAccessor(userData['busId'],'ego_control.brake','time@i,valid@b,brake@d') 
    # 短接部分控制信号 方向盘
    userData['ego_steer'] = BusAccessor(userData['busId'],'ego_control.steer','time@i,valid@b,steer@d') 
    # 构造主车状态总线读取器1
    userData['ego_state'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    # 告警总线
    userData["warning"] = BusAccessor(userData['busId'], "warning", 'time@i,type@b,64@[,text@b')   
    # BSM总线读取器
    userData['V2X_BSM'] = BusAccessor(userData['busId'], 'V2X_BSM.0', 'time@i,100@[,id@i,delaytime@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    # 交通参与物信息读取，type==1为行人,type==0为车辆
    userData['traffic'] = DoubleBusReader(userData['busId'],'traffic','time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f') 
    
# 每个仿真周期(10ms)回调
def ModelOutput(userData):
    # 读取主车状态
    ego_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = userData['ego_state'].readHeader()

    # 读取行人信息
    pedestrians = []
    participant_time, participant_width = userData["traffic"].getReader(userData["time"]).readHeader()   
    for i in range(participant_width):
        id,type,shape,x,y,z,yaw,pitch,roll,speed= userData["traffic"].getReader(userData["time"]).readBody(i)
        if type==1:
            pedestrians.append([id,type,shape,x,y,z,yaw,pitch,roll,speed])
    throttle = 0
    brake = 0

    for i in range(len(pedestrians)):
        Obj_x = pedestrians[i][3]
        Obj_y = pedestrians[i][4]
        Obj_speed = pedestrians[i][9]
        Obj_yaw = pedestrians[i][6]
        longitudinal_offset = abs(ego_x-Obj_x)
        lateral_offset = abs(ego_y-Obj_y)
        distance = math.sqrt(math.pow(longitudinal_offset,2)+math.pow(lateral_offset,2))
        
        #把traffic的坐标转成主车ego坐标
        obj_yaw = (360-Obj_yaw+90)
        if obj_yaw>=360:
            obj_yaw -= 360   
                            
        y_axle_speed_offset = ego_speed*math.sin(ego_yaw) - Obj_speed*math.sin(obj_yaw/180*3.14159)
        x_axle_speed_offset = ego_speed*math.cos(ego_yaw) - Obj_speed*math.cos(obj_yaw/180*3.14159)
        V_error = math.sqrt(math.pow(y_axle_speed_offset,2)+math.pow(x_axle_speed_offset,2))
        TTC = distance/V_error
        print('ttc',TTC,V_error,lateral_offset) 

        if  distance < 25 and lateral_offset < 3.8:#判断行人位置及距离
            if (3 < TTC < 5) or distance < 20:
                throttle = 0
                brake = 1.5
                Warning(userData, 2, 'VRUCW')
            elif TTC < 3 or distance < 10:
                throttle = 0
                brake = 5
                Warning(userData, 2, 'VRUCW')
            # 短接部分控制信号 油门、刹车
            userData['ego_throttle'].writeHeader(*(userData['time'], 1, throttle))
            userData['ego_brake'].writeHeader(*(userData['time'], 1, brake))

# 仿真实验结束时回调
def ModelTerminate(userData):
    pass
