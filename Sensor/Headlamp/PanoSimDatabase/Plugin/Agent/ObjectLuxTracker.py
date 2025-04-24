from DataInterfacePython import *
import matplotlib.pyplot as plt

GT_OBJECT_FORMAT = "time@i,100@[,id@i,type@b,shape@i,range_center@f,range_bbox@f,azimuth_angle@f,elevation_angle@f,velocity@f,heading@f"
OBJECTLUXTRACKER_FORMAT = "time@i,100@[,type@b,id@i,origin@b,x@d,y@d,z@d"
OBJECTLUXVALUE_FORMAT = "time@i,100@[,type@b,id@i,x@d,y@d,z@d,intensity@d,theta@d"

def ModelStart(userData):
    userData["object"] = BusAccessor(userData["busId"], 'GroundTruth_Objects.0', GT_OBJECT_FORMAT)
    userData["objectLuxTracker"] = BusAccessor(userData["busId"], "ObjectLuxTracker", OBJECTLUXTRACKER_FORMAT)
    userData["objectLuxValue"] = BusAccessor(userData["busId"], "ObjectLuxValue", OBJECTLUXVALUE_FORMAT)

    userData["last"] = 0
    userData["x"] = []
    userData["y"] = []

    plt.ion()
    userData["fig"], userData["ax"] = plt.subplots()
    userData["plot"], = userData["ax"].plot([], [])
    userData["ax"].autoscale(True)

def ModelOutput(userData):
    _, width = userData["object"].readHeader()
    index = 0
    for i in range(width):
        id, type, *_ = userData["object"].readBody(i)
        if type == 1:
            userData["objectLuxTracker"].writeBody(index, type, id, 0, 0, 0, 0)
            index += 1
            userData["objectLuxTracker"].writeBody(index, type, id, 0, 0, 0, 1)
            index += 1
            if index >= 50:
                break
    userData["objectLuxTracker"].writeBody(index, 3, 0, 0, -10, -1, 2)
    index += 1
    userData["objectLuxTracker"].writeBody(index, 3, 0, 0, -20, -1, 1)
    index += 1
    userData["objectLuxTracker"].writeHeader(userData["time"], index)

    ts, width = userData["objectLuxValue"].readHeader()
    if ts > userData["last"]:
        userData["last"] = ts
        userData["x"].append(ts)
        if len(userData["x"]) > 100:
            userData["x"].pop(0)
            userData["y"].pop(0)

        if width > 0:
            i = 0
            _, _, _, _, _, intensity, theta = userData["objectLuxValue"].readBody(i)
        else:
            intensity = 0
        userData["y"].append(intensity)

    userData["plot"].set_data(userData["x"], userData["y"])
    userData["ax"].relim()
    userData["ax"].autoscale_view()
    userData["fig"].canvas.flush_events()

def ModelTerminate(userData):
    plt.ioff()
    plt.close()