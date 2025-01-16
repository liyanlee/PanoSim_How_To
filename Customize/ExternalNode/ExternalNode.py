import os
import time
import mmap
import struct
import math
from enum import IntEnum

class Status(IntEnum):
    Unknown         = 0
    Ready           = 1
    Running         = 2
    Paused          = 3
    Error           = 4
    StartReq        = 101
    StopReq         = 102
    ExitReq         = 103
    ClockStartReq   = 201
    ClockPauseReq   = 202
    ClockContinueReq= 203
    ClockStopReq    = 204

ManagerBusStatusOffset      = 0
ManagerBusExperimentOffset  = 1

def wait_experiment_run(bus_manager, timeout=30):
    loop = 0
    run_status = [Status.Ready, Status.StartReq, Status.Running, Status.Paused]
    while loop < timeout:
        status = bus_manager[ManagerBusStatusOffset]
        if status in run_status:
            return True
        time.sleep(1)
        loop += 1
        print('waiting for run experiment: {}s'.format(loop))
    return False

def simulation_step(timestamp, bus_ego):
    ego_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = struct.unpack_from('<iddddddd', bus_ego)
    ego_data = '{}, x={:.2f}, y={:.2f}, yaw={:.2f}°'.format(timestamp, ego_x, ego_y, math.degrees(ego_yaw))
    if timestamp % 1000 == 0:
        print(ego_data)

def main():
    MANAGER_BUS_SIZE = 1024
    busId = 0
    bus_manager = mmap.mmap(fileno = -1, length=MANAGER_BUS_SIZE, tagname='panosim.%d.manager' % busId)
    bus_ego = mmap.mmap(fileno = -1, length=60, tagname='panosim.%d.ego' % busId)

    runnging = wait_experiment_run(bus_manager)
    if runnging:
        name = bus_manager[ManagerBusExperimentOffset:].split(b'\0', 1)[0].decode()
        print('Experiment:', os.environ["PanoSimDatabaseHome"] + "/Experiment/" + name + ".experiment.xml")

    last_time = 0
    while runnging:
        status = bus_manager[ManagerBusStatusOffset]
        if status == Status.Running:
            timestamp = int.from_bytes(bus_ego[0:4], byteorder="little")
            if timestamp > last_time:
                simulation_step(timestamp, bus_ego)
                last_time = timestamp
            else:
                time.sleep(0.001)
        else:
            if status == Status.StartReq:
                print('Status:Start Request')
                last_time = 0
            elif status == Status.Paused:
                print('Status:Paused, time={}'.format(last_time))
            elif status == Status.StopReq:
                print('Status:Stop Request, time={}'.format(last_time))
            elif status == Status.ExitReq:
                print('Status:Exit Request, time={}'.format(last_time))
            elif status == Status.ClockStartReq:
                print('Status:Clock Start Request')
            elif status == Status.ClockPauseReq:
                print('Status:Clock Pause Request, time={}'.format(last_time))
            elif status == Status.ClockContinueReq:
                print('Status:Clock Continue Request, time={}'.format(last_time))
            elif status == Status.ClockStopReq:
                print('Status:Clock Stop Request, time={}'.format(last_time))
            elif status == Status.Ready:
                print('Status:Ready')
            else:
                if status == Status.Error:
                    print('Status:Error')
                elif status == Status.Unknown:
                    print('Status:Unknown')
                else:
                    print('Status:', int(status))
                break
            time.sleep(1)

    print('ExternalNode quit')

if __name__ == "__main__":
    main()
