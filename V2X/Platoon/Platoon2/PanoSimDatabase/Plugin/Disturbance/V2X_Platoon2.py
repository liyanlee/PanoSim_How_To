#  场景描述：车辆组队
from TrafficModelInterface import *


def ModelStart(userData):
    #初始化将参数全部传递给UserData，output中进行逻辑判断选择赋值
    userData['counter'] = 0
    userData['abletochangelane'] = 5
    userData['init_poisition'] = [-112.5,-1620]
    userData['leader_speed'] = 30
    userData['platoon_num'] = 5
    userData['platoon'] = []
    userData['platoon2'] = []
    userData['onturning'] = {}
    userData['init_id'] = addVehicle(userData['init_poisition'][0], userData['init_poisition'][1], 0 , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal)
    print('init_id',userData['init_id'])
    
    print('开始协作式车辆编队应用仿真2：车辆组队...')


def ModelOutput(userData, true=1):
    if len(userData['onturning']) >0:
        for k,v in userData['onturning'].items():
            if v < userData['time']:
                del userData['onturning'][k]
    print('onturning****************************',len(userData['onturning']),userData['onturning'])
    
    userData['counter'] += 1
        
    if userData['counter'] == 10: 
        leader_id = addVehicleRelated(userData['init_id'], -15, 3.5, userData['leader_speed'],  lane=lane_type.current , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal)
        if leader_id not in userData['platoon']:
            userData['platoon'].append(leader_id)
        print('leader_id',leader_id)
        
    for i in range(userData['platoon_num']-1):
        if userData['counter'] == 10:
            follower_id = addVehicleRelated(userData['init_id'], -15*(i+2), 3.5 , userData['leader_speed'],  lane=lane_type.current , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal)
            if follower_id not in userData['platoon']:
                userData['platoon'].append(follower_id)
                print('follower_id',follower_id)
    
    for i in range(userData['platoon_num']-1):
        if userData['counter'] == 10:
            follower2_id = addVehicleRelated(userData['init_id'], -15*(i+2), 0 , userData['leader_speed'],  lane=lane_type.current , type=vehicle_type.OtherVehicle, shape=134, driver=driver_type.normal)
            if follower2_id not in userData['platoon2']:
                userData['platoon2'].append(follower2_id)
                print('follower2_id',follower2_id)
    
    if userData['counter'] > 10:             
        deleteVehicle(userData['init_id'])    #删除定位车辆

    for i in range(len(userData['platoon'])):
        if getDistanceToLaneEnd(userData['platoon'][i]) < 3.0:
            changeRoute(userData['platoon'][i], route_type.straight)
        changeLane(userData['platoon'][i], change_lane_direction.straight, duration=3)
        leader_speed = getVehicleSpeed(userData['platoon'][0])
        changeSpeed(userData['platoon'][i], leader_speed, 3)
        
    for i in range(len(userData['platoon2'])):
        if getDistanceToLaneEnd(userData['platoon2'][i]) < 3.0:
            changeRoute(userData['platoon2'][i], route_type.straight)
 
        leader_speed = getVehicleSpeed(userData['platoon'][0])
        if getLongitudinalDistance(userData['platoon2'][0], userData['platoon'][0]) > 0-(15*userData['platoon_num']):             
            changeSpeed(userData['platoon2'][i], leader_speed-5, 3)
            if userData['platoon2'][i] not in  userData['onturning'].keys():
                changeLane(userData['platoon2'][i], change_lane_direction.straight, duration=3)
        else:
            if getLongitudinalDistance(userData['platoon2'][0], userData['platoon'][-1]) < -15:
                changeSpeed(userData['platoon2'][i], leader_speed+3, 3)
            else:
                changeSpeed(userData['platoon2'][i], leader_speed, 3)
            if getLongitudinalDistance(userData['platoon2'][0], userData['platoon'][0]) != -1073741824.0 and userData['abletochangelane']>1:
                userData['abletochangelane'] -= 1
                print('abletochangelane',userData['abletochangelane'], userData['platoon2'][i])
                changeLane(userData['platoon2'][i], change_lane_direction.left, duration=10)
                userData['onturning'][str(userData['platoon2'][i])] = userData['time'] + 10*1000
            else:
                if len(userData['onturning']) == 0:
                    print('abletochangelane',userData['abletochangelane'], userData['platoon2'][i])
                    changeLane(userData['platoon2'][i], change_lane_direction.straight, duration=3)
            
    if userData['counter'] > 10: 
        print('-----------',getLongitudinalDistance(userData['platoon2'][0], userData['platoon'][0]))
    

def ModelTerminate(userData):
    pass
