"""
LKA (Lane Keeping Assist) Algorithm Plugin

Implements the LKA algorithm based on MathWorks reference:
https://ww2.mathworks.cn/help/releases/R2022a/mpc/ug/lane-keeping-assist-with-lane-detection.html

Algorithm:
  - Stanley Controller for path tracking
  - Inputs: GroundTruth_Objects (obstacle avoidance), LaneInfoPerception (lane detection)
  - Outputs: desired steering angle via xDriver_steer_input bus
  - Real-time metrics visualization with matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from DataInterfacePython import *

# ============================================================================
# LKA 算法参数 (严格按照 MathWorks 页面)
# ============================================================================

# Stanley 控制器参数
K_STEER       = 0.8    # 转向增益
K_CROSSTRACK  = 1.0    # 横向偏差增益
K_HEADING     = 3.0    # 航向角偏差增益

# 预瞄距离 (m)
LOOKAHEAD_DISTANCE = 8.0

# 转向角限制 (rad)
STEER_MAX  =  0.5
STEER_MIN  = -0.5
STEER_RATE =  0.05   # 最大转向速率 (rad/s)

# 安全横向偏移阈值 (m)
SAFE_LATERAL_OFFSET = 1.0

# LKA 激活最低速度 (m/s)
V_MIN_ACTIVATE = 5.0

# 低通滤波系数
STEER_FILTER_ALPHA = 0.3


# ============================================================================
# 可视化配置
# ============================================================================

PLOT_COLORS = ["#FF4444", "#4488FF", "#44BB44", "#9933FF", "#FFB800", "#00CCAA"]
MAX_PLOT_POINTS = 500


# ============================================================================
# ModelStart: 初始化总线和绘图
# ============================================================================

LANE_INFO_FORMAT = (
    "time@i,4@[,"
    "Lane_ID@i,Lane_Distance@d,Lane_Car_Distance_Left@d,Lane_Car_Distance_Right@d,"
    "Lane_Curvature@d,Lane_Coefficient_C0@d,Lane_Coefficient_C1@d,"
    "Lane_Coefficient_C2@d,Lane_Coefficient_C3@d,Lane_Class@b"
)


def ModelStart(userData):
    # --- LaneInfoPerception 总线 (车道线感知输入) ---
    userData["lane_info"] = BusAccessor(
        userData["busId"],
        "LaneInfoPerception.0",
        LANE_INFO_FORMAT
    )

    # --- 主车状态总线 ---
    userData["ego_state"] = BusAccessor(
        userData["busId"],
        "ego",
        "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d"
    )

    # --- 期望转向角输出总线 ---
    userData["steer_output"] = BusAccessor(
        userData["busId"],
        "xDriver_road_wheel_angle_input",
        "time@i,valid@b,road_wheel_angle@d"
    )

    # --- 告警总线 ---
    userData["warning"] = BusAccessor(
        userData["busId"],
        "warning",
        "time@i,type@b,64@[,text@b"
    )

    # ========================================================================
    # 实时曲线初始化
    # ========================================================================
    enable_plot = userData["parameters"].get("show_graph", "FALSE") == "TRUE"
    userData["enable_plot"] = enable_plot

    if enable_plot:
        plt.ion()
        fig = plt.figure("LKA Metrics", figsize=(12, 8), dpi=100)
        fig.clear()
        userData["fig"] = fig

        axes_cfg = [
            ("Lateral Deviation (m)",    0.40, 0.52, 0.55, 0.40),
            ("Steering Angle (deg)",     0.40, 0.07, 0.55, 0.38),
            ("Heading Error (rad)",      0.05, 0.52, 0.30, 0.40),
            ("LKA State / Speed",        0.05, 0.07, 0.30, 0.38),
        ]
        userData["axes_list"] = []
        for title, x, y, w, h in axes_cfg:
            ax = fig.add_axes([x, y, w, h])
            ax.set_title(title, fontsize=11, fontweight="bold")
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, 10)
            userData["axes_list"].append(ax)

        userData["lines"] = {}
        ax_lat, ax_steer, ax_head, ax_state = userData["axes_list"]

        line_lat, = ax_lat.plot([], [], color=PLOT_COLORS[0], lw=2, label="e1 (lateral)")
        line_lat_des, = ax_lat.plot([], [], color=PLOT_COLORS[3], lw=1.5,
                                     linestyle="--", label="lane_center")
        line_lat_left, = ax_lat.plot([], [], color="#888888", lw=1,
                                     linestyle=":", label="left_bound")
        line_lat_right, = ax_lat.plot([], [], color="#888888", lw=1,
                                      linestyle=":", label="right_bound")

        line_steer, = ax_steer.plot([], [], color=PLOT_COLORS[1], lw=2,
                                     label="steer_cmd")
        line_steer_raw, = ax_steer.plot([], [], color=PLOT_COLORS[2], lw=1.5,
                                        linestyle="--", label="steer_raw")
        line_steer_max, = ax_steer.plot([], [], color="#888888", lw=0.8,
                                        linestyle="-", label="limit")

        line_head, = ax_head.plot([], [], color=PLOT_COLORS[4], lw=2,
                                  label="e2 (heading)")
        line_curv, = ax_head.plot([], [], color=PLOT_COLORS[5], lw=1.5,
                                  linestyle="--", label="curvature")

        line_state, = ax_state.plot([], [], color="#9933FF", lw=2,
                                    drawstyle="steps-post")
        line_v_ego, = ax_state.plot([], [], color=PLOT_COLORS[1], lw=1.5,
                                    linestyle="--", label="v_ego")

        userData["lines"]["lat"]       = line_lat
        userData["lines"]["lat_des"]   = line_lat_des
        userData["lines"]["lat_left"]  = line_lat_left
        userData["lines"]["lat_right"] = line_lat_right
        userData["lines"]["steer"]     = line_steer
        userData["lines"]["steer_raw"] = line_steer_raw
        userData["lines"]["steer_max"] = line_steer_max
        userData["lines"]["head"]      = line_head
        userData["lines"]["curv"]      = line_curv
        userData["lines"]["state"]     = line_state
        userData["lines"]["v_ego"]     = line_v_ego

        ax_lat.legend(loc="upper right", fontsize=8)
        ax_steer.legend(loc="upper right", fontsize=8)
        ax_head.legend(loc="upper right", fontsize=8)
        ax_state.legend(loc="upper right", fontsize=8)

        ax_state.set_yticks([0, 1])
        ax_state.set_yticklabels(["Inactive", "Active"], fontsize=8)

        fig.canvas.draw()
        userData["bg"] = fig.canvas.copy_from_bbox(fig.bbox)

    # ========================================================================
    # 数据存储
    # ========================================================================
    userData["time_hist"]       = []
    userData["lat_hist"]        = []
    userData["lat_des_hist"]    = []
    userData["left_hist"]       = []
    userData["right_hist"]     = []
    userData["steer_hist"]      = []
    userData["steer_raw_hist"]  = []
    userData["head_hist"]       = []
    userData["curv_hist"]       = []
    userData["state_hist"]      = []
    userData["v_ego_hist"]     = []
    userData["last_steer"]      = 0.0
    userData["last_time"]       = 0
    userData["lka_active"]      = False


# ============================================================================
# 车道线处理 (从 LaneInfoPerception 总线读取)
# ============================================================================

def read_lane_info(userData):
    """
    从 LaneInfoPerception 总线读取车道线数据

    Bus 格式:
      Header: time, count
      Body[count]: Lane_ID, Lane_Distance, Lane_Car_Distance_Left,
                   Lane_Car_Distance_Right, Lane_Curvature,
                   C0, C1, C2, C3, Lane_Class

    下标固定顺序: 0=左左, 1=左, 2=右, 3=右右

    返回: list of lane_data dict
    """
    _, num_lanes = userData["lane_info"].readHeader()
    lanes = []
    for i in range(num_lanes):
        (_, lane_dist, left_dist, right_dist,
         curvature, c0, c1, c2, c3, lane_class) = userData["lane_info"].readBody(i)
        lanes.append({
            "index":    i,
            "dist":     lane_dist,
            "left_dist": left_dist,
            "right_dist": right_dist,
            "curvature": curvature,
            "c0": c0, "c1": c1, "c2": c2, "c3": c3,
            "cls":      lane_class,
        })
    return lanes


def get_target_lane(lanes):
    """
    从车道线列表中识别本车道左右边界

    下标固定顺序: 0=左左, 1=左, 2=右, 3=右右

    返回: (left_lane, right_lane) 其中 lane = {curvature, c0, c1, c2, c3}
          或 None
    """
    left_lane  = lanes[1] if len(lanes) > 1 else None  # 下标1=左
    right_lane = lanes[2] if len(lanes) > 2 else None  # 下标2=右
    return left_lane, right_lane


def calc_lane_center_at_lookahead(left, right, x=LOOKAHEAD_DISTANCE):
    """
    计算预瞄距离处的车道中心 y 坐标

    车道线用三次多项式描述:
      y = C0 + C1*x + C2*x^2 + C3*x^3

    传感器坐标系: x=纵向(前方+), y=横向(左正右负)
    ego 坐标系:   x=纵向, y=横向(左负右正，与传感器相反)
    """
    def eval_lane(lane, xi):
        if lane is None:
            return 0.0
        c0, c1, c2, c3 = lane["c0"], lane["c1"], lane["c2"], lane["c3"]
        return c0 + c1 * xi + c2 * xi * xi + c3 * xi * xi * xi

    y_left  = eval_lane(left,  x)
    y_right = eval_lane(right, x)

    # 传感器坐标系: 左正右负; ego 坐标系: 左负右正
    # 坐标系转换: ego_y = -sensor_y
    y_left_ego  = -y_left
    y_right_ego = -y_right

    # 车道中心
    lane_center = (y_left_ego + y_right_ego) / 2.0
    return lane_center, y_left_ego, y_right_ego


def calc_curvature_at_lookahead(left, right, x=LOOKAHEAD_DISTANCE):
    """
    计算预瞄距离处的平均曲率

    三次多项式 y = C0 + C1*x + C2*x^2 + C3*x^3:
      y'  = C1 + 2*C2*x + 3*C3*x^2
      y'' = 2*C2 + 6*C3*x
      k   = |y''| / (1 + y'^2)^(3/2)
    """
    def get_curvature(lane):
        if lane is None:
            return 0.0
        c1, c2, c3 = lane["c1"], lane["c2"], lane["c3"]
        y_prime  = c1 + 2 * c2 * x + 3 * c3 * x * x
        y_dprime = 2 * c2 + 6 * c3 * x
        denom    = (1 + y_prime ** 2) ** 1.5
        if abs(denom) < 1e-9:
            return 0.0
        return abs(y_dprime) / denom

    k_left  = get_curvature(left)
    k_right = get_curvature(right)
    return (k_left + k_right) / 2.0


def get_ego_lateral_offset(lanes):
    """
    从 LaneInfoPerception 计算 ego 相对车道中心的横向偏移

    Lane_Car_Distance_Left:  ego 到左车道线的距离 (m)
    Lane_Car_Distance_Right: ego 到右车道线的距离 (m)
    下标: 1=左车道线, 2=右车道线
    ego 横向偏移 = Lane_Car_Distance_Left - Lane_Car_Distance_Right
                 (正值 = ego 偏左, 负值 = ego 偏右)
    """
    left_lane  = lanes[1] if len(lanes) > 1 else None
    right_lane = lanes[2] if len(lanes) > 2 else None

    if left_lane is None or right_lane is None:
        return None

    # ego 在左车道线右侧、右车道线左侧
    # 横向偏移: 左正右负 (传感器坐标系), ego 坐标系反转
    # ego 偏左 -> ego_y < 0 (ego 坐标系)
    # Lane_Car_Distance_Left > Lane_Car_Distance_Right -> ego 偏左
    offset_sensor = left_lane["left_dist"] - right_lane["right_dist"]
    # ego 坐标系: 左负右正
    offset_ego = -offset_sensor
    return offset_ego


# ============================================================================
# Stanley 控制器 (LKA 核心算法)
# ============================================================================

def stanley_controller(v_ego, ego_yaw, e1, e2, curvature):
    """
    Stanley 控制器 (基于 MathWorks LKA 参考)

    控制律:
      delta = e2 + arctan(K * e1 / v) + curvature * v^2 / g

    参数:
      e1: 横向偏差 (lateral deviation)
      e2: 相对横摆角 (relative yaw angle)
      curvature: 道路曲率 (1/m)
    """
    if abs(v_ego) < 0.01:
        return 0.0

    # 航向误差项
    heading_term = K_HEADING * e2

    # 横向误差项 (Stanley 核心)
    crosstrack_term = np.arctan2(K_CROSSTRACK * e1, max(v_ego, 0.5))

    # 曲率前馈项
    g_gravity = 9.81
    curv_term = curvature * (v_ego ** 2) / g_gravity

    delta_raw = heading_term + crosstrack_term + curv_term
    return delta_raw


def clip_steer_rate(delta_new, delta_old, dt, v_ego):
    """限制转向速率"""
    if dt <= 0:
        return delta_new
    rate     = (delta_new - delta_old) / dt
    max_rate = STEER_RATE * (1.0 + 0.5 * v_ego / 20.0)
    if rate > max_rate:
        delta_new = delta_old + max_rate * dt
    elif rate < -max_rate:
        delta_new = delta_old - max_rate * dt
    return delta_new


def check_lka_activation(e1, v_ego, safe_offset=SAFE_LATERAL_OFFSET):
    """判断是否需要激活 LKA"""
    if v_ego < V_MIN_ACTIVATE:
        return False
    return abs(e1) > safe_offset


# ============================================================================
# ModelOutput: 主循环
# ============================================================================

def ModelOutput(userData):
    sim_time = userData["time"]
    t_sec    = sim_time / 1000.0
    dt       = (sim_time - userData.get("last_time", sim_time)) / 1000.0
    userData["last_time"] = sim_time

    # ========================================================================
    # 1. 读取主车状态
    # ========================================================================
    _, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed \
        = userData["ego_state"].readHeader()

    # ego_speed: PanoSim 通常为 m/s
    # ego_speed = ego_speed / 3.6  # 如需 km/h -> m/s 转换

    # ========================================================================
    # 2. 读取 LaneInfoPerception 车道线
    # ========================================================================
    lanes = read_lane_info(userData)

    if not lanes:
        # 无车道线数据
        userData["steer_output"].writeHeader(sim_time, 0, userData["last_steer"])
        return

    # ========================================================================
    # 3. 车道线解析
    # ========================================================================
    left_lane, right_lane = get_target_lane(lanes)

    if left_lane is None and right_lane is None:
        userData["steer_output"].writeHeader(sim_time, 0, userData["last_steer"])
        return

    # 预瞄距离处的车道中心
    lane_center, left_bound, right_bound = calc_lane_center_at_lookahead(
        left_lane, right_lane, LOOKAHEAD_DISTANCE
    )

    # 预瞄距离处的曲率
    curvature = calc_curvature_at_lookahead(left_lane, right_lane, LOOKAHEAD_DISTANCE)

    # ego 横向偏移 (ego 坐标系)
    ego_lateral_offset = get_ego_lateral_offset(lanes)

    if ego_lateral_offset is None:
        ego_lateral_offset = 0.0

    # ========================================================================
    # 4. 计算状态误差
    # ========================================================================
    # e1: 横向偏差 = ego 横向位置 - 车道中心
    # ego_y: ego 在全局坐标系下的横向位置 (m)
    # lane_center: ego 坐标系下的车道中心 (m)
    # MathWorks 坐标系: ego_y = 0 表示车道中心
    # 转换: e1 = ego_y - lane_center (ego_y 已是全局坐标，需要转换)
    #
    # 更直接的方式: 用 LaneInfoPerception 提供的左右距离差计算
    # left_dist - right_dist > 0: ego 偏左 (需要右转修正)
    e1 = ego_lateral_offset

    # e2: 相对横摆角 = 道路切线角 (path_yaw 本身已在车辆坐标系下)
    # path_yaw = curvature * lookahead, 表示道路相对于车辆朝向的偏角
    path_yaw = curvature * LOOKAHEAD_DISTANCE
    e2 = path_yaw
    e2 = np.arctan2(np.sin(e2), np.cos(e2))  # 归一化到 [-pi, pi]

    # ========================================================================
    # 5. LKA 激活判断
    # ========================================================================
    lka_active = check_lka_activation(e1, ego_speed)

    if lka_active:
        # Stanley 控制器
        delta_raw = stanley_controller(ego_speed, ego_yaw, e1, e2, curvature)

        # 限制转向角范围
        delta_clamped = np.clip(delta_raw, STEER_MIN, STEER_MAX)

        # 限制转向速率
        delta_smooth = clip_steer_rate(
            delta_clamped, userData["last_steer"], dt, ego_speed
        )

        # 低通滤波平滑
        alpha = STEER_FILTER_ALPHA
        delta_final = alpha * delta_smooth + (1 - alpha) * userData["last_steer"]

        userData["last_steer"] = delta_final
    else:
        delta_final = 0.0
        delta_raw = 0.0
        userData["last_steer"] = 0.0

    # ========================================================================
    # 6. 写入 xDriver_steer_input 总线
    # ========================================================================
    valid = 1 if lka_active else 0
    # 左正右负: Stanley 输出正=右转, 取反变为左正右负
    road_wheel_angle = np.degrees(-delta_final)
    userData["steer_output"].writeHeader(sim_time, valid, road_wheel_angle)

    # ========================================================================
    # 7. 写入 Warning 总线
    # ========================================================================
    if abs(e1) > SAFE_LATERAL_OFFSET * 2.0:
        warning_text = b"LKA_CRITICAL"
        userData["warning"].writeHeader(sim_time, 1, len(warning_text))
        userData["warning"].getBus()[9:9+len(warning_text)] = warning_text
    elif abs(e1) > SAFE_LATERAL_OFFSET:
        warning_text = b"LKA_WARNING"
        userData["warning"].writeHeader(sim_time, 1, len(warning_text))
        userData["warning"].getBus()[9:9+len(warning_text)] = warning_text

    # ========================================================================
    # 8. 打印状态
    # ========================================================================
    state_str = "Active" if lka_active else "Inactive"
    print(f"[LKA] t={t_sec:.2f}s  State={state_str}  "
          f"v_ego={ego_speed:.1f}m/s  e1={e1:.3f}m  e2={e2:.3f}rad  "
          f"curv={curvature:.4f}(1/m)  steer={np.degrees(delta_final):.2f}deg  "
          f"lane_center={lane_center:.2f}m  lanes={len(lanes)}")

    # ========================================================================
    # 9. 实时曲线更新
    # ========================================================================
    if userData.get("enable_plot", False):
        _update_plot(userData, t_sec, e1, lane_center, left_bound, right_bound,
                     delta_final, delta_raw, e2, curvature, lka_active, ego_speed)


def _update_plot(userData, t_sec, e1, lane_c, left_b, right_b,
                  steer, steer_raw, e2, curv, active, v_ego):
    """更新 matplotlib 实时曲线"""
    userData["time_hist"].append(t_sec)
    userData["lat_hist"].append(e1)
    userData["lat_des_hist"].append(lane_c)
    userData["left_hist"].append(left_b)
    userData["right_hist"].append(right_b)
    userData["steer_hist"].append(steer)
    userData["steer_raw_hist"].append(steer_raw)
    userData["head_hist"].append(e2)
    userData["curv_hist"].append(curv)
    userData["state_hist"].append(1.0 if active else 0.0)
    userData["v_ego_hist"].append(v_ego)

    max_pts = MAX_PLOT_POINTS
    hist_keys = ["time_hist", "lat_hist", "lat_des_hist", "left_hist",
                 "right_hist", "steer_hist", "steer_raw_hist",
                 "head_hist", "curv_hist", "state_hist", "v_ego_hist"]
    if len(userData["time_hist"]) > max_pts:
        for key in hist_keys:
            userData[key].pop(0)

    times = userData["time_hist"]
    t_max = max(times[-1] if times else 10.0, 10.0)
    t_min = max(0.0, t_max - 15.0)

    fig = userData["fig"]
    fig.canvas.restore_region(userData["bg"])

    ax_lat, ax_steer, ax_head, ax_state = userData["axes_list"]

    # Lateral Deviation
    line_lat       = userData["lines"]["lat"]
    line_lat_des   = userData["lines"]["lat_des"]
    line_lat_left  = userData["lines"]["lat_left"]
    line_lat_right = userData["lines"]["lat_right"]
    line_lat.set_data(times, userData["lat_hist"])
    line_lat_des.set_data(times, userData["lat_des_hist"])
    line_lat_left.set_data(times, userData["left_hist"])
    line_lat_right.set_data(times, userData["right_hist"])
    ax_lat.set_xlim(t_min, t_max)
    lat_vals = userData["lat_hist"]
    lat_max = max(abs(v) for v in lat_vals) if lat_vals else 1.0
    ax_lat.set_ylim(-max(2.0, lat_max * 1.5), max(2.0, lat_max * 1.5))
    ax_lat.axhline(0, color="gray", lw=0.8)
    ax_lat.axhline( SAFE_LATERAL_OFFSET, color="#FF4444", lw=0.8, linestyle="--")
    ax_lat.axhline(-SAFE_LATERAL_OFFSET, color="#FF4444", lw=0.8, linestyle="--")
    for artist in [line_lat, line_lat_des, line_lat_left, line_lat_right]:
        ax_lat.draw_artist(artist)
    ax_lat.legend(loc="upper right", fontsize=8)

    # Steering Angle
    line_steer   = userData["lines"]["steer"]
    line_steer_r = userData["lines"]["steer_raw"]
    line_steer_m = userData["lines"]["steer_max"]
    line_steer.set_data(times, [np.degrees(v) for v in userData["steer_hist"]])
    line_steer_r.set_data(times, [np.degrees(v) for v in userData["steer_raw_hist"]])
    line_steer_m.set_data(times, [np.degrees(STEER_MAX)] * len(times))
    ax_steer.set_xlim(t_min, t_max)
    ax_steer.set_ylim(np.degrees(STEER_MIN) - 2, np.degrees(STEER_MAX) + 2)
    ax_steer.axhline(0, color="gray", lw=0.8)
    ax_steer.axhline( np.degrees(STEER_MAX), color="#888888", lw=0.5)
    ax_steer.axhline( np.degrees(STEER_MIN), color="#888888", lw=0.5)
    for artist in [line_steer, line_steer_r, line_steer_m]:
        ax_steer.draw_artist(artist)
    ax_steer.legend(loc="upper right", fontsize=8)

    # Heading Error
    line_head = userData["lines"]["head"]
    line_curv = userData["lines"]["curv"]
    line_head.set_data(times, userData["head_hist"])
    curv_scaled = [v * 10.0 for v in userData["curv_hist"]]
    line_curv.set_data(times, curv_scaled)
    ax_head.set_xlim(t_min, t_max)
    head_vals = [v for v in userData["head_hist"] if abs(v) < 100]
    ax_head.set_ylim(-max(0.5, max(abs(v) for v in head_vals) * 1.5 if head_vals else 0.5),
                     max(0.5, max(abs(v) for v in head_vals) * 1.5 if head_vals else 0.5))
    ax_head.axhline(0, color="gray", lw=0.8)
    for artist in [line_head, line_curv]:
        ax_head.draw_artist(artist)
    ax_head.legend(loc="upper right", fontsize=8)

    # State
    line_state = userData["lines"]["state"]
    line_v_ego = userData["lines"]["v_ego"]
    line_state.set_data(times, userData["state_hist"])
    v_scaled = [v / 40.0 for v in userData["v_ego_hist"]]
    line_v_ego.set_data(times, v_scaled)
    ax_state.set_xlim(t_min, t_max)
    ax_state.set_ylim(-0.5, 1.5)
    for artist in [line_state, line_v_ego]:
        ax_state.draw_artist(artist)

    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()


# ============================================================================
# ModelTerminate: 清理
# ============================================================================

def ModelTerminate(userData):
    if userData.get("enable_plot", False):
        plt.ioff()
        plt.close("LKA Metrics")
    print("[LKA] Plugin terminated.")
