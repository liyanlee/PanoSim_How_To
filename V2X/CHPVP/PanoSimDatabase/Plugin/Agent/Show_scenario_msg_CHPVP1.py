from DataInterfacePython import *
 
def Warning(userData,level,warn):
    bus = userData["warning"].getBus()
    size = userData["warning"].getHeaderSize()
    bus[size:size + len(warn)] = '{}'.format(warn).encode()
    userData["warning"].writeHeader(*(userData["time"], level, len(warn)))

# 仿真实验启动时回调
def ModelStart(userData):
    # 构造主车状态总线读取器1
    userData['ego_state'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    # 构造主车状态总线读取器2
    userData['ego_extra'] = BusAccessor(userData['busId'], 'ego_extra',
                                        'time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d')
    # 构建告警总线读取器
    userData["warning"] = BusAccessor(userData['busId'], "warning", 'time@i,type@b,64@[,text@b')   


# 每个仿真周期(10ms)回调 to clean the road
def ModelOutput(userData):
    if userData['time'] / 1000 > 3.0 :
        Warning(userData, 2, 'Fire truck is coming')
    if userData['time'] / 1000 > 5.0 :
        Warning(userData, 2, 'Change trafficlight status')
    if userData['time'] / 1000 > 7.0 :
        Warning(userData, 2, 'Shorten Red light time')
    if userData['time'] / 1000 > 9.0 :
        Warning(userData, 2, 'Fire truck pass')
    if userData['time'] / 1000 > 17 :
        Warning(userData, 0, ' ')
    if userData['time'] / 1000 > 25:
        stopSimulation()
        
def ModelTerminate(userData):
    pass
