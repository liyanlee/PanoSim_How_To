import numpy as np
from DataInterfacePython import *
import math
 
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
    userData["warning"] = BusAccessor(userData['busId'], "warning", 'time@i,type@b,64@[,text@b')   
    # BSM总线读取器
    userData['V2X_BSM'] = BusAccessor(userData['busId'], 'V2X_BSM.0', 'time@i,100@[,id@i,delaytime@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    

# 每个仿真周期(10ms)回调 to clean the road
def ModelOutput(userData):
    if userData['time'] / 1000 > 43.0 :
        Warning(userData, 2, 'Open car drive too slowly')
    if userData['time'] / 1000 > 50.0 :
        Warning(userData, 2, 'No enough time to pass')
    if userData['time'] / 1000 > 56.0 :
        Warning(userData, 2, 'Extend time of east-west direction')
    if userData['time'] / 1000 > 70.0 : 
        Warning(userData, 2, 'Avoid congestion')
    if userData['time'] / 1000 > 80.0 :
        Warning(userData, 0, ' ')

    if userData['time'] / 1000 > 80:
        stopSimulation()



def ModelTerminate(userData):
    pass
