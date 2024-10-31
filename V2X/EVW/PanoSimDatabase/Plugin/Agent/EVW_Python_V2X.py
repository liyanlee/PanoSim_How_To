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

# 每个仿真周期(10ms)回调
def ModelOutput(userData):
    RV_IS_Emergency_Vehicle = True
    # 读取主车状态
    ego_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = userData['ego_state'].readHeader()

    # 读取交通车信息
    obj_attibutes = []
    obj_time, obj_width = userData['V2X_BSM'].readHeader()

    for i in range(obj_width):
        id,delay_time,x,y,z,yaw,pitch,roll,speed = userData['V2X_BSM'].readBody(i)
        OBJ_X = x
        OBJ_Y = y
        OBJ_Re_Vx = speed
        OBJ_Yaw = yaw
        obj_attibutes.append([OBJ_X, OBJ_Y, OBJ_Re_Vx,OBJ_Yaw])

    if obj_width > 0:
        for i in range(obj_width):
            Obj_x = obj_attibutes[i][0]
            Obj_y = obj_attibutes[i][1]
            Obj_speed = obj_attibutes[i][2]
            Obj_yaw = obj_attibutes[i][3]
            longitudinal_offset = abs(ego_x-Obj_x)
            lateral_offset = abs(ego_y-Obj_y)
            distance = math.sqrt(math.pow(longitudinal_offset,2)+math.pow(lateral_offset,2))

            #把traffic的坐标转成主车ego坐标
            obj_yaw = (360-Obj_yaw+90)
            if obj_yaw>=359:
                obj_yaw -= 359
                
            if abs(ego_yaw*180/3.14159-obj_yaw)<5 or abs(360-abs(ego_yaw*180/3.14159-obj_yaw))<5: 
                some_direction = True
            else:
                some_direction = False     
                
            y_axle_speed_offset = ego_speed*math.sin(ego_yaw) - Obj_speed*math.sin(obj_yaw/180*3.14159)
            x_axle_speed_offset = ego_speed*math.cos(ego_yaw) - Obj_speed*math.cos(obj_yaw/180*3.14159)
            V_error = math.sqrt(math.pow(y_axle_speed_offset,2)+math.pow(x_axle_speed_offset,2))
            TTC = distance/V_error
            print('ttc',TTC,V_error)

            if  distance < 30 and some_direction and RV_IS_Emergency_Vehicle and lateral_offset < 3:#判断同向车道的车辆位置
                #TODO  上层车控 --- 换道让行，现在用驾驶员模型Trajectory Fellower实现
                Warning(userData, 2, 'EVW')
            
# 仿真实验结束时回调
def ModelTerminate(userData):
    pass
