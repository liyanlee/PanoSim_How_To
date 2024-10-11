from DataInterfacePython import *

def is_traffic_sign(type):
    return type == 4

def ModelStart(userData):
    sensor_name = 'GroundTruth_Objects.0'
    sensor_output_format = 'time@i,100@[,id@i,type@b,shape@i,range_center@f,range_bbox@f,azimuth_angle@f,elevation_angle@f,velocity@f,heading@f'
    userData['objects'] = BusAccessor(userData['busId'], sensor_name, sensor_output_format)
    userData['last'] = 0

def ModelOutput(userData):
    ts, width = userData['objects'].readHeader()
    if ts > userData['last']:
        userData['last'] = ts
        for i in range(width):
            _, type, shape, _, _, _, _, _, _ = userData['objects'].readBody(i)
            if is_traffic_sign(type):
                if shape == 342:
                    print('分向行驶车道(二股)')
                if shape == 343:
                    print('十字路口标志牌组合(60)')

def ModelTerminate(userData):
    pass
