from TrafficModelInterface import *


def ModelStart(userData):
    userData['init_poisition'] = [-198,-5.4] 
    userData['sv_speed'] = 12.5
    userData['sv_tl_ids'] = []
    userData['sv_tl_id'] = 0
    userData['start_change_time'] = 5*1000
    
    userData['sv_id'] = addVehicle(userData['init_poisition'][0], userData['init_poisition'][1], userData['sv_speed'] , type=vehicle_type.OtherVehicle, shape=11, driver=driver_type.normal)
    
    print('V2X_PanoTown_CHPVP1_Python 开始仿真...')


def ModelOutput(userData, true=1):
    trafficlights = getTrafficLightList()
    
    sv_lane = getVehicleLane(userData['sv_id'])
    sv_lanes = getNextLanes(sv_lane, next_junction_direction.straight)
    sv_edges = []
    for lane in sv_lanes:
        sv_edges.append(getEdgeByLane(lane))

    for id in trafficlights:
        edge = getTrafficLightEdge(id)
        if edge in sv_edges:
            userData['sv_tl_id'] = id
    
    sv_phase = ''   
    if sv_phase != None:
        sv_phase = getTrafficLightPhase(userData['sv_tl_id'], next_junction_direction.straight)        
    for id in trafficlights:
        edge = getTrafficLightEdge(id)
        phase = getTrafficLightPhase(id, next_junction_direction.straight)
        if phase == sv_phase:
            if id not in userData['sv_tl_ids']:
                userData['sv_tl_ids'].append(id)       
         
    red_left_time = int((6 - (userData['time'] - userData['start_change_time'])/1000))
    next_phase_left_time = int((30 - (userData['time'] - 10*1000)/1000))
    
    # 此demo由时间触发，可修改为由VIR、BSM数据包触发 TODO
    if userData['time'] > 5*1000:  
        if userData['time'] <= 10*1000:
            for id in trafficlights:
                if id in userData['sv_tl_ids']:            
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.red , timer=red_left_time)
                else:
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.green , timer=red_left_time)
        else:
            for id in trafficlights:
                if id in userData['sv_tl_ids']:            
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.green , timer=next_phase_left_time-3)
                else:
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.red , timer=next_phase_left_time)
                           
    changeRoute(userData['sv_id'], route_type.straight)
    changeLane(userData['sv_id'], change_lane_direction.straight, duration=3)
    changeSpeed(userData['sv_id'],userData['sv_speed'], 1)

def ModelTerminate(userData):
    pass
