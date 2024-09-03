from DataInterfacePython import *

def ModelStart(userData):
    Format = 'time@i,temperature@f,pressure@f,humidity@f,precipitation_type@b,particle_size@f,particle_capacity@f,\
        falling_alpha@f,falling_beta@f,falling_speed@f,fog_visibility@f,lighting_alpha@f,lighting_beta@f,\
        lighting_intensity@f,lighting_ambient@f,street_light@b,vehicle_light@b,skybox@b,friction@f,wetness@f,snow@f'
    userData['weather'] = BusAccessor(userData['busId'], 'weather', Format)

def get_value_by_time(timestamp, values):
    for ts, value in reversed(values):
        if timestamp > ts:
            return value

def ModelOutput(userData):
    ts = userData['time']
    Temperature = [(0, 16), (5000, 16+(ts-1000)/2000), (20000, 30-(ts-20000)/1000), (45000, 30)]
    PrecipitationType = [(0, 0), (60000, 1), (75000, 2), (90000, 0)]
    ParticleCapacity = [(0, 0), (60000, 10000 + ts/10), (90000, 0)]
    FogVisibility = [(0, 499), (45000, 500-(ts-45000)/10), (50000, 1+(ts-50000)/100)]
    LightingAlpha = [(0, 10), (1000, 10+((ts-1000)/1000)*4), (20000, 85-((ts-20000)/1000)*4), (30000, 0), (90000, 60)]
    LightingBeta = [(0, 30), (10000, 30+(ts-10000)/1000), (30000, 0), (90000, 50)]
    LightingIntensity = [(0, 10000), (1000, 10000+(ts-1000)*3), (20000, 90000-(ts-20000)*4), (30000, 100), (90000, 30000+(ts-90000)*3)]
    LightingAmbient = [(0, 1000), (1000, 1000+(ts-1000)), (20000, 10000-(ts-20000)*0.5), (30000, 1), (60000, 11+(ts-60000)/1000), (90000, 8000)]
    StreetLight = [(0, 0), (30000, 1), (60000, 0)]
    VehicleLight = [(0, 0), (30000, 1), (45000, 0)]
    Skybox = [(0, 0), (30000, 1), (45000, 2), (90000, 0)]
    Wetness = [(0, 0), (60000, (ts-60000)/10000), (70000, 0.95-(ts-70000)/10000), (80000, (ts-80000)/10000), (90000, 0.9-(ts-90000)/10000)]
    Snow = [(0, 0), (75000, (ts-75000)/10000), (90000, 0.8-(ts-90000)/10000)]
    temperature = get_value_by_time(ts, Temperature)
    pressure = 1
    humidity = 60
    precipitation_type = get_value_by_time(ts, PrecipitationType)
    particle_size = 0.5
    particle_capacity = get_value_by_time(ts, ParticleCapacity)
    falling_alpha = 0
    falling_beta = 0
    falling_speed = 0.8
    fog_visibility = get_value_by_time(ts, FogVisibility)
    lighting_alpha = get_value_by_time(ts, LightingAlpha)
    lighting_beta = get_value_by_time(ts, LightingBeta)
    lighting_intensity = get_value_by_time(ts, LightingIntensity)
    lighting_ambient = get_value_by_time(ts, LightingAmbient)
    street_light = get_value_by_time(ts, StreetLight)
    vehicle_light = get_value_by_time(ts, VehicleLight)
    skybox = get_value_by_time(ts, Skybox)
    friction = 0.8
    wetness = get_value_by_time(ts, Wetness)
    snow = get_value_by_time(ts, Snow)
    userData['weather'].writeHeader(*(userData['time'], temperature, pressure, humidity, precipitation_type, particle_size, particle_capacity,
                                      falling_alpha, falling_beta, falling_speed, fog_visibility, lighting_alpha, lighting_beta,
                                      lighting_intensity, lighting_ambient, street_light, vehicle_light, skybox, friction, wetness, snow))

def ModelTerminate(userData):
    pass
