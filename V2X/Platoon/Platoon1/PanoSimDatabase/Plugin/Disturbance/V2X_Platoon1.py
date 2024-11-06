#  场景描述：车队巡航
from TrafficModelInterface import *

def ModelStart(userData):
    #初始化将参数全部传递给UserData，output中进行逻辑判断选择赋值
    userData['counter'] = 0
    userData['abletochangelane'] = 5
    userData['init_poisition'] = [-93,-2000]
    userData['leader_speed'] = 25
    userData['platoon_num'] = 5
    userData['platoon'] = []
    userData['init_id'] = addVehicle(userData['init_poisition'][0], userData['init_poisition'][1], 0 , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal) #创建定位车辆

    print('开始协作式车辆编队应用仿真1：车辆编队巡航...')


def ModelOutput(userData, true=1):
    userData['counter'] += 1
        
    if userData['counter'] == 10: 
        leader_id = addVehicleRelated(userData['init_id'], -15, 0, userData['leader_speed'],  lane=lane_type.current , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal)    #创建头车
        if leader_id not in userData['platoon']:
            userData['platoon'].append(leader_id)
        print('leader_id',leader_id)
        
    for i in range(userData['platoon_num']-1):
        if userData['counter'] == 10:
            follower_id = addVehicleRelated(userData['init_id'], -15*(i+2), 0 , userData['leader_speed'],  lane=lane_type.current , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal)
            if follower_id not in userData['platoon']:
                userData['platoon'].append(follower_id)
                print('follower_id',follower_id)
    
    if userData['counter'] > 10:             
        deleteVehicle(userData['init_id'])   #删除定位车辆
        
    for i in range(len(userData['platoon'])):
        changeRoute(userData['platoon'][i], route_type.straight)
        changeLane(userData['platoon'][i], change_lane_direction.straight, duration=3)
        changeSpeed(userData['platoon'][i], userData['leader_speed'] , 1)
        
        if getLeaderVehicle(userData['platoon'][0])>0:
            dis = getLongitudinalDistance(getLeaderVehicle(userData['platoon'][0]), userData['platoon'][0])           
            if dis < 80 :  #处理同车道前车
                changeSpeed(getLeaderVehicle(userData['platoon'][0]), userData['leader_speed'], 1)
   
        if getFollowerVehicle(userData['platoon'][-1])>0:
            dis = getLongitudinalDistance(userData['platoon'][-1],getFollowerVehicle(userData['platoon'][-1]))         
            if dis < 20 :  #处理同车道后车，后车速度保持与车队一致
                print('*////////////////')                
                changeSpeed(getFollowerVehicle(userData['platoon'][-1]), userData['leader_speed'], 1)


def ModelTerminate(userData):
    pass
