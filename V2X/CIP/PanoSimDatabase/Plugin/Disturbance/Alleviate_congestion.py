import math
from xml.dom.minidom import parseString
import numpy as np
import sys
import json
import random
import mmap
import time
from TrafficModelInterface import *
import os


def ModelStart(userData):
    userData['init_poisition'] = [-90,-1.8]  # [100,1.8]
    userData['lv_speed'] = 10
    userData['lv_speed1'] = 2.5
    userData['lv_speed2'] = 10
    userData['ids'] = []
    userData['lv_tl_id'] = 0
    userData['lv_tl_ids'] = []
    userData['start_change_time'] = 57*1000
    
    
    userData['lv_id'] = addVehicle(userData['init_poisition'][0], userData['init_poisition'][1], userData['lv_speed'] , type=vehicle_type.Car, shape=22, driver=driver_type.normal)
    userData['fv_id1'] = addVehicle(userData['init_poisition'][0]-10, userData['init_poisition'][1], userData['lv_speed'] , type=vehicle_type.OtherVehicle, shape=11, driver=driver_type.normal)
    userData['fv_id2'] = addVehicle(userData['init_poisition'][0]-30, userData['init_poisition'][1], userData['lv_speed'] , type=vehicle_type.Car, shape=23, driver=driver_type.normal)
    userData['fv_id3'] = addVehicle(userData['init_poisition'][0]-40, userData['init_poisition'][1], userData['lv_speed'] , type=vehicle_type.Car, shape=31, driver=driver_type.normal)
    userData['fv_id4'] = addVehicle(userData['init_poisition'][0]-50, userData['init_poisition'][1], userData['lv_speed'] , type=vehicle_type.Car, shape=23, driver=driver_type.normal)
    userData['fv_id5'] = addVehicle(userData['init_poisition'][0]-60, userData['init_poisition'][1], userData['lv_speed'] , type=vehicle_type.Car, shape=31, driver=driver_type.normal)
    userData['ids'] = [userData['lv_id'],userData['fv_id1'],userData['fv_id2'],userData['fv_id3'],userData['fv_id4'],userData['fv_id5']] #
    
    print('ids',userData['ids'])
    
    print('V2X_PanoTown_CIP1_Python 开始仿真...')


def ModelOutput(userData, true=1):
    
    for id in userData['ids']:
        changeRoute(id, route_type.straight)
        changeLane(id, change_lane_direction.straight, duration=3)
    
    if userData['time'] > 40*1000:  
        for id in userData['ids']:
            changeRoute(id, route_type.straight)
            changeLane(id, change_lane_direction.straight, duration=3)
            changeSpeed(id,userData['lv_speed1'], 1)
    
    
    
    # 交通灯控制
    trafficlights = getTrafficLightList()
    
    lv_lane = getVehicleLane(userData['lv_id'])
    lv_lanes = getNextLanes(lv_lane, next_junction_direction.straight)
    lv_edges = []
    for lane in lv_lanes:
        lv_edges.append(getEdgeByLane(lane))
    
    for id in trafficlights:
        edge = getTrafficLightEdge(id)
        if edge in lv_edges:
            userData['lv_tl_id'] = id
    
    lv_phase = ''   
    if lv_phase != None:
        lv_phase = getTrafficLightPhase(userData['lv_tl_id'], next_junction_direction.straight)        
    for id in trafficlights:
        edge = getTrafficLightEdge(id)
        phase = getTrafficLightPhase(id, next_junction_direction.straight)

        if phase == lv_phase:
            if id not in userData['lv_tl_ids']:
                userData['lv_tl_ids'].append(id)   

    # 此demo由时间触发，可修改为由BSM、RSC数据包触发 TODO
    if userData['time'] > 57*1000:  
        red_left_time = int((13 - (userData['time'] - userData['start_change_time'])/1000))
        next_phase_left_time = int((40 - (userData['time'] - 70*1000)/1000))

        if userData['time'] <= 70*1000:
            for id in trafficlights:
                if id in userData['lv_tl_ids']:            
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.yellow , timer=0)#red_left_time
                else:
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.red , timer=red_left_time)
        else:
            for id in trafficlights:
                if id in userData['lv_tl_ids']:            
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.red , timer=next_phase_left_time)
                else:
                    setTrafficLightState(id, next_junction_direction.straight , traffic_light_state.green , timer=next_phase_left_time-3)


def ModelTerminate(userData):
    pass
