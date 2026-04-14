"""
ACC (Adaptive Cruise Control) Algorithm Plugin

Implements the ACC classical controller based on MathWorks Sensor Fusion example:
https://ww2.mathworks.cn/help/releases/R2022a/driving/ug/adaptive-cruise-control-with-sensor-fusion.html

Algorithm (per MathWorks page, section "Adaptive Cruise Controller"):
  - Safe distance: D_safe = D_default + T_gap * V_x
  - Speed controller: a_speed = K_p * (v_setpoint - v_ego), clamped to >= 0 (accel only)
  - Gap controller: a_gap = K_p * (D_safe - d_rel), clamped to <= 0 (brake only)
  - Classical ACC: Switch+Max block -> d_rel>=D_safe uses a_speed, else uses a_gap
  - Acceleration clamped to [-3, 2] m/s^2
  - Real-time metrics visualization with matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from DataInterfacePython import *

# ============================================================================
# ACC 算法参数 (严格按照 MathWorks 页面)
# ============================================================================

# 驾驶员设定速度 (km/h)
V_SETPOINT_DEFAULT = 40

# 安全距离参数 (页面上 D_default 和 T_gap)
# D_safe = D_default + T_gap * V_x
D_DEFAULT = 10.0   # 默认停车间距 (m)
T_GAP     = 1.1    # 时间车头时距 (s)

# 速度控制器 P 增益
K_P_SPEED = 0.5

# 加速度上下限 (m/s^2) — 页面明确写明: [-3, 2] m/s^2
A_MAX =  2.0
A_MIN = -3.0

# ACC 激活最低速度 (m/s)
V_MIN_ACTIVATE = 5.0

# 主车前悬长度 (m)
EGO_FRONT_OVERHANG = 0.75

# ============================================================================
# 可视化配置
# ============================================================================

PLOT_COLORS = ["#FF4444", "#4488FF", "#44BB44", "#9933FF", "#FFB800"]
MAX_PLOT_POINTS = 500


# ============================================================================
# ModelStart: 初始化总线和绘图
# ============================================================================

def ModelStart(userData):
    # --- GroundTruth Objects 总线 (传感器输入) ---
    userData["gt_objects"] = BusAccessor(
        userData["busId"],
        "GroundTruth_Objects.0",
        "time@i,100@[,id@i,type@b,shape@i,range_center@f,range_bbox@f,"
        "azimuth_angle@f,elevation_angle@f,velocity@f,heading@f"
    )

    # --- 主车状态总线 ---
    userData["ego_state"] = BusAccessor(
        userData["busId"],
        "ego",
        "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d"
    )

    # --- 期望加速度/减速度输出总线 ---
    userData["acc_output"] = BusAccessor(
        userData["busId"],
        "xDriver_accel_input",
        "time@i,valid@b,accel@d"
    )

    # ========================================================================
    # 实时曲线初始化
    # ========================================================================
    enable_plot = userData["parameters"].get("show_graph", "FALSE") == "TRUE"
    userData["enable_plot"] = enable_plot

    if enable_plot:
        plt.ion()
        fig = plt.figure("ACC Metrics", figsize=(12, 8), dpi=100)
        fig.clear()
        userData["fig"] = fig

        # 子图: Speed, Gap Distance, Desired Acceleration, ACC State
        axes_cfg = [
            ("Ego Speed (m/s)",        0.40, 0.52, 0.55, 0.40),
            ("Desired Accel (m/s²)",   0.40, 0.07, 0.55, 0.38),
            ("Gap Distance (m)",       0.05, 0.52, 0.30, 0.40),
            ("ACC State / Lead Vehicle", 0.05, 0.07, 0.30, 0.38),
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
        ax_spd, ax_acc, ax_gap, ax_state = userData["axes_list"]

        line_spd, = ax_spd.plot([], [], color=PLOT_COLORS[1], lw=2, label="v_ego")
        line_vset, = ax_spd.plot([], [], color=PLOT_COLORS[3], lw=1.5,
                                  linestyle="--", label="v_setpoint")
        line_vlead, = ax_spd.plot([], [], color=PLOT_COLORS[0], lw=1.5,
                                   linestyle=":", label="v_lead")

        line_acc, = ax_acc.plot([], [], color=PLOT_COLORS[2], lw=2, label="a_desired")
        line_acc_spd, = ax_acc.plot([], [], color=PLOT_COLORS[3], lw=1.5,
                                     linestyle="--", label="a_speed")
        line_acc_gap, = ax_acc.plot([], [], color=PLOT_COLORS[4], lw=1.5,
                                     linestyle=":", label="a_gap")
        line_amin, = ax_acc.plot([], [], color="#888888", lw=0.8,
                                  linestyle="-", label="a_min")

        line_gap, = ax_gap.plot([], [], color=PLOT_COLORS[4], lw=2, label="G (actual)")
        line_gap_des, = ax_gap.plot([], [], color="#888888", lw=1.5,
                                      linestyle="--", label="G_des")

        line_state, = ax_state.plot([], [], color="#9933FF", lw=2,
                                     drawstyle="steps-post")

        userData["lines"]["spd"]     = line_spd
        userData["lines"]["vset"]     = line_vset
        userData["lines"]["vlead"]   = line_vlead
        userData["lines"]["acc"]     = line_acc
        userData["lines"]["acc_spd"] = line_acc_spd
        userData["lines"]["acc_gap"] = line_acc_gap
        userData["lines"]["amin"]     = line_amin
        userData["lines"]["gap"]     = line_gap
        userData["lines"]["gap_des"] = line_gap_des
        userData["lines"]["state"]   = line_state

        ax_spd.legend(loc="upper right", fontsize=8)
        ax_acc.legend(loc="upper right", fontsize=8)
        ax_gap.legend(loc="upper right", fontsize=8)
        ax_state.set_yticks([0, 1])
        ax_state.set_yticklabels(["SpeedCtrl", "GapCtrl"], fontsize=8)

        fig.canvas.draw()
        userData["bg"] = fig.canvas.copy_from_bbox(fig.bbox)

    # ========================================================================
    # 数据存储
    # ========================================================================
    userData["time_hist"]    = []
    userData["spd_hist"]     = []
    userData["vset_hist"]    = []
    userData["vlead_hist"]   = []
    userData["acc_hist"]     = []
    userData["acc_spd_hist"] = []
    userData["acc_gap_hist"] = []
    userData["dist_hist"]    = []
    userData["d_safe_hist"]  = []
    userData["state_hist"]   = []
    userData["acc_state"]    = 0   # 0=SpeedCtrl, 1=GapCtrl


# ============================================================================
# 选择前车 (MathWorks findLeadCar 逻辑)
# ============================================================================

# 车道参数 (MathWorks 默认)
LANE_WIDTH      = 3.6    # 标准车道宽 (m)
HALF_LANE_WIDTH  = LANE_WIDTH / 2.0  # = 1.8 m
MAX_LEAD_RANGE   = 500.0  # 最大检测范围 (m)


def select_lead_vehicle(objects, ego_speed, ego_yaw):
    """
    从 GroundTruth Objects 中选择前车 (MathWorks findLeadCar 逻辑)

    车道模型 (MathWorks):
      - 车道用抛物线 y = ax^2 + bx + c 描述, a=0 (直道), c=±1.8m
      - ego 位于车道正中 (横向偏移 0)
      - 左车道边界: y_left  = HALF_LANE_WIDTH  = +1.8m
      - 右车道边界: y_right = -HALF_LANE_WIDTH = -1.8m
      - 本车道内: y_right <= y <= y_left

    坐标变换:
      - angle   = -azimuth_angle         (传感器坐标 → ego 坐标系)
      - x       = range_center * cos(angle)   (纵向位置, ego 前方为正)
      - y       = range_center * sin(angle)   (横向位置, ego 左正右负)
      - yaw     = π/2 - heading
      - deltaYaw = yaw - ego_yaw
      - v_long  = velocity * cos(deltaYaw)    (目标纵向速度)
      - v_rel   = v_long - ego_speed          (相对纵向速度, >0 = 接近)

    选择: 纵向距离 x 最小的本车道车辆
    """
    best_x = MAX_LEAD_RANGE
    best   = None

    for obj in objects:
        (id_, type_, _, range_center, _,
         azimuth, elevation, velocity, heading) = obj

        # 只考虑车辆 (type == 0)
        if type_ != 0:
            continue

        if range_center <= 0.0 or range_center > MAX_LEAD_RANGE:
            continue

        # 坐标变换 (与 MATLAB findLeadCar 完全一致)
        angle    = -azimuth
        yaw      = np.pi / 2.0 - heading
        deltaYaw = yaw - ego_yaw
        x        = range_center * np.cos(angle)  # 纵向位置 (m)
        y        = range_center * np.sin(angle)  # 横向位置 (m)

        # 车道边界 (直道, a=0, b=0)
        y_left  =  HALF_LANE_WIDTH   # +1.8 m
        y_right = -HALF_LANE_WIDTH   # -1.8 m

        # 本车道过滤: y_right <= y <= y_left
        if y_right <= y <= y_left:
            # 选择纵向距离 x 最小的前车 (减去主车前悬长度)
            if x < best_x:
                best_x = x
                v_long = velocity * np.cos(deltaYaw)
                v_rel  = v_long - ego_speed
                d_rel  = x - EGO_FRONT_OVERHANG
                best   = {
                    "id":          id_,
                    "x":           x,       # 纵向距离 (原始)
                    "d_rel":       d_rel,   # 纵向距离 (减去前悬)
                    "y":           y,       # 横向偏移
                    "velocity":    velocity,
                    "v_long":      v_long,
                    "v_rel":       v_rel,
                }

    if best is None:
        return None

    return best


# ============================================================================
# ACC 控制计算
# ============================================================================

def calc_safe_distance(v_ego):
    """
    计算安全跟车距离 (MathWorks 页面公式):
    D_safe = D_default + T_gap * V_x
    """
    return D_DEFAULT + T_GAP * v_ego


def acc_controller(v_ego, v_setpoint, d_rel):
    """
    ACC 经典控制器 (MathWorks 页面 "Adaptive Cruise Controller" 部分)

    速度控制回路 (Speed Control):
      a_speed = K_p * (v_setpoint - v_ego)

    距离控制回路 (Gap Control):
      D_safe = D_default + T_gap * v_ego
      a_gap  = K_p * (d_rel - D_safe)

    经典控制器逻辑 (Switch Block):
      - d_rel < D_safe  (距离危险): 使用 Gap Controller  (a_gap ∈ [-3, 0], 减速)
      - d_rel >= D_safe (距离安全): 使用 Speed Controller (a_speed ∈ [0, 2], 加速)

    Speed Controller: a_speed = K_p * (v_setpoint - v_ego), Saturation [0, 2]
    Gap Controller:   a_gap   = K_p * (d_rel - D_safe),     Saturation [-3, 0]
    Switch Block:     根据 d_rel 与 D_safe 关系选择其一

    最终加速度: a_desired = clamp(selected, A_MIN, A_MAX)
    """
    # ACC 激活条件
    if v_ego < V_MIN_ACTIVATE:
        return 0.0, 0, float("inf"), float("inf"), 0.0, 0.0

    # 安全距离: D_safe = D_default + T_gap * v_ego
    d_safe = calc_safe_distance(v_ego)

    if d_rel is None or d_rel >= 1e6:
        # 无前车或距离异常: 纯速度控制
        # a_speed = K_p * (v_setpoint - v_ego), clamp 到 >= 0 (只加速)
        a_speed = max(K_P_SPEED * (v_setpoint - v_ego), 0.0)
        a_desired = np.clip(a_speed, A_MIN, A_MAX)
        return a_desired, 0, float("inf"), float("inf"), a_speed, 0.0

    # Gap Controller: a_gap = K_p * (d_rel - D_safe)
    #   d_rel < D_safe (危险): 输出 < 0 (减速)
    #   d_rel > D_safe (安全): 输出 > 0 (加速追车)
    # MathWorks Saturation 块: 下限 A_MIN (-3), 上限 0 (只减速不加速)
    a_gap_raw = K_P_SPEED * (d_rel - d_safe)
    a_gap = max(min(a_gap_raw, 0.0), A_MIN)  # clamp 到 [-3, 0]

    # Speed Controller: a_speed = K_p * (v_setpoint - v_ego)
    # MathWorks Saturation 块: 下限 0, 上限 A_MAX (2)
    a_spd_raw = K_P_SPEED * (v_setpoint - v_ego)
    a_speed = min(max(a_spd_raw, 0.0), A_MAX)  # clamp 到 [0, 2]

    # Switch Block (MathWorks 核心逻辑):
    #   - d_rel < D_safe (距离危险): 选 a_gap (GapCtrl, 负值, 减速)
    #   - d_rel >= D_safe (距离安全): 选 a_speed (SpeedCtrl, 正值, 加速)
    if d_rel < d_safe:
        a_raw = a_gap      # GapCtrl 生效: 输出 [-3, 0]
        new_state = 1
    else:
        a_raw = a_speed    # SpeedCtrl 生效: 输出 [0, 2]
        new_state = 0

    # 最终约束到 [-3, 2] m/s^2
    a_desired = np.clip(a_raw, A_MIN, A_MAX)

    return a_desired, new_state, d_rel, d_safe, a_speed, a_gap


# ============================================================================
# ModelOutput: 主循环
# ============================================================================

def ModelOutput(userData):
    sim_time = userData["time"]
    t_sec    = sim_time / 1000.0

    # ========================================================================
    # 1. 读取主车状态
    # ========================================================================
    _, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed \
        = userData["ego_state"].readHeader()

    # ========================================================================
    # 2. 读取 GroundTruth Objects
    # ========================================================================
    _, num_objects = userData["gt_objects"].readHeader()
    objects = []
    for i in range(num_objects):
        obj = userData["gt_objects"].readBody(i)
        objects.append(obj)

    # ========================================================================
    # 3. 选择前车 (MathWorks findLeadCar 逻辑)
    # ========================================================================
    lead = select_lead_vehicle(objects, ego_speed, ego_yaw)

    d_rel   = lead["d_rel"]    if lead else None  # 纵向距离 (已减去前悬)
    v_lead  = lead["v_long"]  if lead else 0.0    # 前车纵向速度

    # ========================================================================
    # 4. ACC 控制器
    # ========================================================================
    # 从参数读取设定速度 (m/s)
    v_setpoint = float(userData["parameters"].get("v_setpoint", str(V_SETPOINT_DEFAULT))) / 3.6

    a_desired, new_state, d_actual, d_safe, a_speed, a_gap = \
        acc_controller(ego_speed, v_setpoint, d_rel)

    userData["acc_state"] = new_state

    # ========================================================================
    # 5. 写入 xDriver_accel_input 总线
    # ========================================================================
    valid = 1 if ego_speed >= V_MIN_ACTIVATE else 0
    userData["acc_output"].writeHeader(sim_time, valid, a_desired)

    # ========================================================================
    # 6. 打印状态
    # ========================================================================
    state_names = ["SpeedCtrl", "GapCtrl"]
    if lead is not None:
        print(f"[ACC] t={t_sec:.2f}s  State={state_names[new_state]}  "
              f"v_ego={ego_speed:.1f}m/s  v_set={v_setpoint:.1f}m/s  "
              f"v_lead={v_lead:.1f}m/s  a_des={a_desired:.2f}m/s²  "
              f"d_rel={d_actual:.1f}m  D_safe={d_safe:.1f}m")
    else:
        print(f"[ACC] t={t_sec:.2f}s  State=SpeedCtrl  "
              f"v_ego={ego_speed:.1f}m/s  v_set={v_setpoint:.1f}m/s  "
              f"a_des={a_desired:.2f}m/s²")

    # ========================================================================
    # 7. 实时曲线更新
    # ========================================================================
    if userData.get("enable_plot", False):
        _update_plot(userData, t_sec, ego_speed, v_setpoint, v_lead,
                     a_desired, a_speed, a_gap, d_actual, d_safe, new_state)


def _update_plot(userData, t_sec, v_ego, v_set, v_lead,
                 a_des, a_spd, a_gap, d_act, d_saf, state):
    """更新 matplotlib 实时曲线"""
    userData["time_hist"].append(t_sec)
    userData["spd_hist"].append(v_ego)
    userData["vset_hist"].append(v_set)
    userData["vlead_hist"].append(v_lead if v_lead > 0 else float("nan"))
    userData["acc_hist"].append(a_des)
    userData["acc_spd_hist"].append(a_spd)
    userData["acc_gap_hist"].append(a_gap if a_gap != float("inf") else float("nan"))
    userData["dist_hist"].append(d_act if d_act != float("inf") else float("nan"))
    userData["d_safe_hist"].append(d_saf if d_saf != float("inf") else float("nan"))
    userData["state_hist"].append(state)

    max_pts = MAX_PLOT_POINTS
    hist_keys = ["time_hist", "spd_hist", "vset_hist", "vlead_hist",
                 "acc_hist", "acc_spd_hist", "acc_gap_hist",
                 "dist_hist", "d_safe_hist", "state_hist"]
    if len(userData["time_hist"]) > max_pts:
        for key in hist_keys:
            userData[key].pop(0)

    times = userData["time_hist"]
    t_max = max(times[-1] if times else 10.0, 10.0)
    t_min = max(0.0, t_max - 15.0)

    fig = userData["fig"]
    fig.canvas.restore_region(userData["bg"])

    ax_spd, ax_acc, ax_gap, ax_state = userData["axes_list"]

    # Speed
    line_spd   = userData["lines"]["spd"]
    line_vset  = userData["lines"]["vset"]
    line_vlead = userData["lines"]["vlead"]
    line_spd.set_data(times, userData["spd_hist"])
    line_vset.set_data(times, userData["vset_hist"])
    line_vlead.set_data(times, userData["vlead_hist"])
    ax_spd.set_xlim(t_min, t_max)
    spd_max = max(userData["spd_hist"]) if userData["spd_hist"] else 40.0
    vset_max = max(userData["vset_hist"]) if userData["vset_hist"] else 40.0
    ax_spd.set_ylim(0, max(40.0, spd_max, vset_max) * 1.2)
    for artist in [line_spd, line_vset, line_vlead]:
        ax_spd.draw_artist(artist)
    ax_spd.legend(loc="upper right", fontsize=8)

    # Acceleration
    line_acc    = userData["lines"]["acc"]
    line_acc_spd = userData["lines"]["acc_spd"]
    line_acc_gap = userData["lines"]["acc_gap"]
    line_amin   = userData["lines"]["amin"]
    line_acc.set_data(times, userData["acc_hist"])
    line_acc_spd.set_data(times, userData["acc_spd_hist"])
    line_acc_gap.set_data(times, userData["acc_gap_hist"])
    line_amin.set_data(times, [A_MIN] * len(times))
    ax_acc.set_xlim(t_min, t_max)
    acc_vals = [v for v in userData["acc_hist"] if v > -1e9]
    acc_min = min(A_MIN, min(acc_vals) if acc_vals else A_MIN)
    acc_max = max(A_MAX, max(acc_vals) if acc_vals else A_MAX)
    ax_acc.set_ylim(acc_min - 1.0, acc_max + 1.0)
    ax_acc.axhline(0, color="gray", lw=0.8)
    for artist in [line_acc, line_acc_spd, line_acc_gap, line_amin]:
        ax_acc.draw_artist(artist)
    ax_acc.legend(loc="upper right", fontsize=8)

    # Distance
    line_dist    = userData["lines"]["gap"]
    line_d_saf   = userData["lines"]["gap_des"]
    line_dist.set_data(times, userData["dist_hist"])
    line_d_saf.set_data(times, userData["d_safe_hist"])
    ax_gap.set_xlim(t_min, t_max)
    d_vals = [v for v in userData["dist_hist"] if v == v]  # filter nan
    ds_vals = [v for v in userData["d_safe_hist"] if v == v]
    d_max = max(d_vals or [40.0])
    ds_max = max(ds_vals or [40.0])
    ax_gap.set_ylim(0, max(40.0, d_max, ds_max) * 1.2)
    for artist in [line_dist, line_d_saf]:
        ax_gap.draw_artist(artist)
    ax_gap.legend(loc="upper right", fontsize=8)

    # State
    line_state = userData["lines"]["state"]
    line_state.set_data(times, userData["state_hist"])
    ax_state.set_xlim(t_min, t_max)
    ax_state.set_ylim(-0.5, 1.5)
    ax_state.draw_artist(line_state)

    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()


# ============================================================================
# ModelTerminate: 清理
# ============================================================================

def ModelTerminate(userData):
    if userData.get("enable_plot", False):
        plt.ioff()
        plt.close("ACC Metrics")
    print("[ACC] Plugin terminated.")
