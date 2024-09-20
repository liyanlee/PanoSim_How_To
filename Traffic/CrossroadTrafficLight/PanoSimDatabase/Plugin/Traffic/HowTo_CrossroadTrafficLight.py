from TrafficModelInterface import *

def ModelStart(userData):
	userData['stop_vehicles'] = {}

def ModelOutput(userData):
	EgoId, group = 0, userData['time'] // 10 % 10
	for id in getVehicleList():
		if id % 10 == group and id != EgoId:
			if id in userData['stop_vehicles']:
				if isInternalLane(getVehicleLane(id)):
					userData['stop_vehicles'].pop(id)
				else:
					if getTrafficLightState(id, getRoute(id)) == traffic_light_state.green:
						if isStopped(id):
							resumeVehicle(id)
						else:
							cancelStopVehicle(id)
						userData['stop_vehicles'].pop(id)
			else:
				distance = getDistanceToLaneEnd(id)
				if distance < 50 and distance >= 0:
					light_state = getTrafficLightState(id, getRoute(id))
					if light_state == traffic_light_state.red or light_state == traffic_light_state.yellow:
						stopVehicle(id, distance)
						userData['stop_vehicles'][id] = 0

def ModelTerminate(userData):
	pass