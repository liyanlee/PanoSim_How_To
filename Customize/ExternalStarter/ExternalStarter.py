import os, time, mmap, subprocess, win32event
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

class Const:
    ManagerSize             = 1024
    ManagerStatusOffset     = 0
    ManagerExperimentOffset = 1
    EgoSize                 = 60

# 根据运行场景修改
RunnerManagerPath   = 'D:/PanoSim5/PanoSim5_release/Bin/py36'
ExperimentName      = 'HowToExternalStarter'

def check_environment():
    runner = os.path.join(RunnerManagerPath, 'RunnerManager.exe')
    if not os.path.exists(runner):
        raise FileNotFoundError('RunnerManager.exe not found:', runner)
    ExperimentFile = os.path.join(os.environ.get("PanoSimDatabaseHome", ''),
                                  'Experiment', ExperimentName, 'Temp.experiment.xml')
    if not os.path.exists(ExperimentFile):
        raise FileNotFoundError('Experiment file not found:', ExperimentFile)
    return runner

def start_experiment(runner):
    manager = mmap.mmap(fileno=-1, length=Const.ManagerSize, tagname="panosim.0.manager")
    if manager[Const.ManagerStatusOffset] == Status.Unknown:
        subprocess.Popen([runner, '0'], cwd=RunnerManagerPath)
        while manager[Const.ManagerStatusOffset] == Status.Unknown:
            print('waiting for RunnerManager.exe running')
            time.sleep(0.1)
    if manager[Const.ManagerStatusOffset] == Status.Ready:
        exp = str.encode(ExperimentName + "/Temp\0")
        manager[Const.ManagerExperimentOffset:Const.ManagerExperimentOffset+len(exp)] = exp
        manager[Const.ManagerStatusOffset] = Status.StartReq
    while manager[Const.ManagerStatusOffset] != Status.Running:
        print('waiting for experiment({}) running'.format(ExperimentName))
        time.sleep(0.1)
    return manager

def stop_experiment(manager):
    if manager[Const.ManagerStatusOffset] == Status.Running:
        manager[Const.ManagerStatusOffset] = Status.StopReq
    while True:
        status = manager[Const.ManagerStatusOffset]
        if status == Status.Ready or status == Status.Unknown or status == Status.Error:
            break
        time.sleep(1)

def experiment_running(run_time=5):
    timestamp = 0
    event = win32event.CreateEvent(None, True, False, "panosim.0.step")
    ego = mmap.mmap(fileno=-1, length=Const.EgoSize, tagname='panosim.0.ego')
    while timestamp < run_time * 1000:
        win32event.WaitForSingleObject(event, 10)
        timestamp = int.from_bytes(ego[0:4], byteorder="little")
        if timestamp % 1000 == 0:
            print('current time:', timestamp)

def main():
    try:
        runner = check_environment()
        manager = start_experiment(runner)
        experiment_running()
        stop_experiment(manager)
    except Exception as e:
        print(e)
    finally:
        print('ExternalStarter quit')

if __name__ == "__main__":
    main()
