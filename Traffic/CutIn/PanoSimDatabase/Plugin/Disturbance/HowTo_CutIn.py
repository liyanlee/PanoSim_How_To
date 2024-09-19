from TrafficModelInterface import *

def ModelStart(userData):
    userData['cut_in_id'] = -1
    userData['step_index'] = 0
    userData['Steps'] = ['depart', 'cut-in', 'keep-run', 'delete', 'done']

def ModelOutput(userData):
    EgoId, cut_in_id = 0, userData['cut_in_id']
    DepartTime, DepartStationToEgo, DepartLateralToEgo, DepartSpeed, DepartLaneToEgo = 1000, 5, 0, 5, lane_type.right
    CutInTime, CutInSpeed, CutInDirection = DepartTime + 2000, 5, change_lane_direction.left
    Direction2Route = [route_type.straight, route_type.left, route_type.right, route_type.turn_round, route_type.unknown]
    if userData['Steps'][userData['step_index']] == 'depart':
        if userData['time'] >= DepartTime:
            cut_in_id = addVehicleRelated(EgoId, DepartStationToEgo, DepartLateralToEgo, DepartSpeed, DepartLaneToEgo)
            if cut_in_id > 0:
                userData['cut_in_id'] = cut_in_id
                userData['step_index'] += 1
    elif userData['Steps'][userData['step_index']] == 'cut-in':
        if userData['time'] > CutInTime:
            changeSpeed(cut_in_id, float(CutInSpeed), 1)
            changeLane(cut_in_id, CutInDirection)
            userData['step_index'] += 1
    elif userData['Steps'][userData['step_index']] == 'keep-run':
        changeSpeed(cut_in_id, float(CutInSpeed), 1)
        if getRoute(cut_in_id) == next_junction_direction.unknown:
            directions = getValidDirections(getVehicleLane(cut_in_id))
            if directions:
                changeRoute(cut_in_id, Direction2Route[int(directions[0])])
        if getVehicleSpeed(EgoId) < 0.1:
            userData['step_index'] += 1
    elif userData['Steps'][userData['step_index']] == 'delete':
        deleteVehicle(cut_in_id)
        userData['step_index'] += 1

def ModelTerminate(userData):
    pass
