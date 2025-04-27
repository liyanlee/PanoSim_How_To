from DataInterfacePython import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import LinearSegmentedColormap

MAX_LUX = 150
WIDTH = 1200
HEIGHT = 1200

def mouse_move(event):
    x, y = event.xdata, event.ydata
    if x and y:
        plt.title("Lux: %f" % lux_data[HEIGHT-int(y)][int(x)])

def ModelStart(userData):
    OutputFormat = "time@i,%d@[,intensity@f" % (1200 * 1200)
    userData['sensor'] = BusAccessor(userData['busId'], 'LuxMeter.0', OutputFormat)
    userData['last_time'] = 0

    data = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    data[0][0] = np.log10(1 + MAX_LUX / 1000)

    plt.ion()
    colors = ["slateblue", "skyblue", "lime", "yellow", "red"]
    my_cmap = LinearSegmentedColormap.from_list("my_cmap", colors)
    userData["fig"] = plt.figure()
    userData["im"] = plt.imshow(data, aspect='equal', cmap=my_cmap)
    plt.connect('motion_notify_event', mouse_move)

def ModelOutput(userData):
    global lux_data
    ts, _ = userData['sensor'].readHeader()
    if ts > userData["last_time"]:
        userData["last_time"] = ts
        data = np.frombuffer(userData['sensor'].getBus(), dtype=np.float32, offset=8)
        data = data.reshape(HEIGHT, WIDTH)
        lux_data = data
        data[data == 0] = np.NaN
        data = np.log10(1 + data / 1000)
        data = np.flip(data, axis=0)
        userData["im"].set_data(data)
        # plt.contour(data, levels=[np.log10(1 + 1 / 1000), np.log10(1 + 5 / 1000)])
        userData["fig"].canvas.flush_events()

def ModelTerminate(userData):
    plt.ioff()
    plt.close()
