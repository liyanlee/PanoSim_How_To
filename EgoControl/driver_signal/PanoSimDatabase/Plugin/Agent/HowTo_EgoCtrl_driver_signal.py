from DataInterfacePython import *
import keyboard

def ModelStart(userData):
    userData["ego_control"] = BusAccessor(userData["busId"], "ego_control", "time@i,valid@b,throttle@d,brake@d,steer@d,mode@i,gear@i")
    userData["mode"] = 6

def ModelOutput(userData):
    throttle, brake, steer = 0, 0, 0
    if keyboard.is_pressed('up'):
        throttle = 0.3
    if keyboard.is_pressed('down'):
        brake = 5
    if keyboard.is_pressed('left'):
        steer = 180
    if keyboard.is_pressed('right'):
        steer = -180
    if keyboard.is_pressed('page up'):
        userData["mode"] = 6
    if keyboard.is_pressed('page down'):
        userData["mode"] = -1
    Valid, Gear = 1, 0
    userData["ego_control"].writeHeader(*(userData["time"], Valid, float(throttle), float(brake), float(steer), userData["mode"], Gear))

def ModelTerminate(userData):
    pass
