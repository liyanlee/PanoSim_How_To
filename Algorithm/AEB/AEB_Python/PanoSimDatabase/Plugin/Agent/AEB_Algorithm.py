"""
AEB (Autonomous Emergency Braking) Algorithm Plugin

Implements the AEB algorithm based on MathWorks Sensor Fusion example:
https://ww2.mathworks.cn/help/releases/R2022a/driving/ug/autonomous-emergency-braking-with-sensor-fusion.html

Algorithm:
  - TTC (Time-To-Collision) based decision logic
  - FCW → PB1 (Partial Brake 1) → PB2 (Partial Brake 2) → FB (Full Brake)
  - Uses GroundTruth_Objects as sensor input
  - Outputs desired acceleration
  - Real-time metrics visualization with matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from DataInterfacePython import *

# ============================================================================
# AEB 算法参数 (与 MathWorks 参考一致)
# ============================================================================

# 驾驶员反应时间 (s)
TAU_REACT = 1.2

# 驾驶员舒适减速度 (m/s^2)
A_DRIVER = 4.0

# AEB 最大减速度 (m/s^2)
A_AEB = 10.0

# 主车前悬长度 (m)
EGO_FRONT_OVERHANG = 0.75

# TTC 阈值 (s)
# TTC_THRESH_FCW = TAU_REACT + v_ego / A_DRIVER  (动态计算, 不使用固定值)
TTC_THRESH_PB1  = 1.4   # Partial Brake Stage 1
TTC_THRESH_PB2  = 0.6   # Partial Brake Stage 2
TTC_THRESH_FB   = 0.3   # Full Brake: TTC<=0.3s 时全力制动


# ============================================================================
# 可视化配置
# ============================================================================

PLOT_METRICS = ["TTC", "Deceleration", "Ego Speed", "TTC Thresholds"]
PLOT_COLORS  = ["#FF4444", "#4488FF", "#44BB44", "#888888"]
MAX_PLOT_POINTS = 500   # 曲线最大点数，超过后滚动丢弃旧数据


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
    userData["aeb_output"] = BusAccessor(
        userData["busId"],
        "xDriver_accel_input",
        "time@i,valid@b,accel@d"
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
        fig = plt.figure("AEB Metrics", figsize=(12, 8), dpi=100)
        fig.clear()
        userData["fig"] = fig

        # 子图: TTC, Deceleration, Ego Speed, Relative Distance
        axes_cfg = [
            ("TTC (s)",              0.40, 0.52, 0.55, 0.40),
            ("Deceleration (m/s²)",  0.40, 0.07, 0.55, 0.38),
            ("Ego Speed (m/s)",      0.05, 0.52, 0.30, 0.40),
            ("TTC Threshold States", 0.05, 0.07, 0.30, 0.38),
        ]
        userData["axes_list"] = []
        for i, (title, x, y, w, h) in enumerate(axes_cfg):
            ax = fig.add_axes([x, y, w, h])
            ax.set_title(title, fontsize=11, fontweight="bold")
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.grid(True, alpha=0.3)
            if "Speed" in title or "Decel" in title:
                ax.set_xlim(0, 10)
            else:
                ax.set_xlim(0, 10)
            userData["axes_list"].append(ax)

        # 预创建 line 对象
        userData["lines"] = {}
        ax_ttc, ax_dec, ax_spd, ax_state = userData["axes_list"]

        line_ttc, = ax_ttc.plot([], [], color=PLOT_COLORS[0], lw=2, label="TTC")
        line_thr, = ax_ttc.plot([], [], color=PLOT_COLORS[3], lw=1.5,
                                 linestyle="--", label="TTC Thresholds")
        line_dec, = ax_dec.plot([], [], color=PLOT_COLORS[1], lw=2, label="a_desired")
        line_spd, = ax_spd.plot([], [], color=PLOT_COLORS[2], lw=2, label="v_ego")
        line_state, = ax_state.plot([], [], color="#9933FF", lw=2, drawstyle="steps-post")
        line_rdist, = ax_spd.plot([], [], color="#FFB800", lw=1.5,
                                   linestyle=":", label="d_rel (m)")

        userData["lines"]["ttc"]      = line_ttc
        userData["lines"]["thr"]      = line_thr
        userData["lines"]["dec"]      = line_dec
        userData["lines"]["spd"]      = line_spd
        userData["lines"]["rdist"]    = line_rdist
        userData["lines"]["state"]    = line_state

        ax_ttc.legend(loc="upper right", fontsize=8)
        ax_dec.legend(loc="upper right", fontsize=8)
        ax_spd.legend(loc="upper right", fontsize=8)
        ax_state.set_yticks([0, 1, 2, 3, 4])
        ax_state.set_yticklabels(["None", "FCW", "PB1", "PB2", "FB"], fontsize=8)

        fig.canvas.draw()
        userData["bg"] = fig.canvas.copy_from_bbox(fig.bbox)

    # ========================================================================
    # 数据存储
    # ========================================================================
    userData["time_hist"]     = []
    userData["ttc_hist"]     = []
    userData["dec_hist"]     = []
    userData["spd_hist"]     = []
    userData["dist_hist"]    = []
    userData["state_hist"]   = []
    userData["aeb_state"]    = 0   # 0=None, 1=FCW, 2=PB1, 3=PB2, 4=FB
    userData["last_aeb_state"] = 0


# ============================================================================
# TTC 计算 (核心算法)
# ============================================================================

def calc_ttc(d_rel, v_rel):
    """
    计算 TTC (Time-To-Collision)

    TTC = d_rel / v_rel

    d_rel > 0: 前方目标
    v_rel > 0: ego 接近目标 (gap 缩小)
    v_rel <= 0: ego 未接近目标或目标更快

    Returns:
        ttc: Time-To-Collision (s), 正数表示碰撞时间, inf 表示无碰撞风险
    """
    if d_rel <= 0.0:
        return float("inf")
    if v_rel <= 0.0:
        return float("inf")
    ttc = d_rel / v_rel
    if ttc < 0.0:
        return float("inf")
    return ttc


def calc_stopping_time(v_ego, a_brake):
    """
    计算车辆制动停车时间: τ_stop = v_ego / |a_brake|
    """
    if a_brake <= 0.0:
        return float("inf")
    return v_ego / a_brake


# ============================================================================
# AEB 决策状态机
# ============================================================================

def aeb_decision(v_ego, d_rel, v_rel, current_state):
    """
    根据 MathWorks 参考实现 AEB 决策逻辑

    停车总时间: τ_FCW = τ_react + v_ego / A_DRIVER
    TTC 与各阈值比较, 决定 AEB 状态:
      0 = None (无干预)
      1 = FCW  (Forward Collision Warning, 警告)
      2 = PB1  (Partial Brake Stage 1, 部分制动1)
      3 = PB2  (Partial Brake Stage 2, 部分制动2)
      4 = FB   (Full Brake, 全力制动)

    单向状态机: 一旦进入制动阶段 (state>=2), 状态只能向前转移,
    不得回退到 FCW 或 None
    """
    if d_rel <= 0.0:
        # 目标消失: 制动阶段保持当前状态
        if current_state >= 2:
            return current_state, float("inf")
        return 0, float("inf")

    if v_ego < 0.1:
        return current_state, float("inf")

    ttc = calc_ttc(d_rel, v_rel)
    if ttc == float("inf"):
        # 安全: 制动阶段保持当前状态
        if current_state >= 2:
            return current_state, ttc
        return 0, ttc

    # 动态 TTC 阈值 (基于当前速度)
    tau_stop_driver = calc_stopping_time(v_ego, A_DRIVER)
    ttc_thr_fcw = TAU_REACT + tau_stop_driver  # FCW 阈值

    # 状态判决
    new_state = 0
    if ttc <= TTC_THRESH_FB:
        new_state = 4  # Full Brake
    elif ttc <= TTC_THRESH_PB2:
        new_state = 3  # PB2
    elif ttc <= TTC_THRESH_PB1:
        new_state = 2  # PB1
    elif ttc <= ttc_thr_fcw:
        new_state = 1  # FCW

    # 单向状态机: PB1 及以上不退出, 只能向前
    if current_state >= 2:
        return max(current_state, new_state), ttc
    # FCW 可以退出
    return new_state, ttc


# ============================================================================
# 减速度计算
# ============================================================================

def calc_desired_accel(state, v_ego):
    """
    根据 AEB 状态计算期望减速度

    MathWorks 参考:
      - PB1: -4 m/s^2 (部分制动)
      - PB2: -6 m/s^2 (增强部分制动)
      - FB:  -10 m/s^2 (全力制动)
    """
    if state == 0:  # 无干预
        return 0.0
    elif state == 1:  # FCW (纯警告, 不加制动)
        return 0.0
    elif state == 2:  # PB1
        return -4.0
    elif state == 3:  # PB2
        return -6.0
    else:  # FB (Full Brake)
        return -A_AEB


# ============================================================================
# 选择最危险目标 (纵向, 即 ego 前方)
# ============================================================================

# 车道参数
LANE_WIDTH      = 3.6    # 标准车道宽 (m)
HALF_LANE_WIDTH  = LANE_WIDTH / 2.0  # = 1.8 m
MAX_LEAD_RANGE   = 500.0  # 最大检测范围 (m)


def select_most_dangerous(objects, ego_speed, ego_yaw):
    """
    从 GroundTruth Objects 中选择最危险目标 (TTC 最小)

    过滤条件 (参考 ACC findLeadCar 逻辑):
      - type == 0: 车辆
      - 通过角度和车道宽度判断是否在本车道前方
      - range_center > 0 (前方)
    """
    candidates = []

    for obj in objects:
        (id_, type_, _, range_center, range_bbox,
         azimuth, elevation, velocity, heading) = obj

        # 只考虑车辆 (type=0)
        if type_ != 0:
            continue

        if range_center <= 0.0 or range_center > MAX_LEAD_RANGE:
            continue

        # 坐标变换 (与 ACC findLeadCar 一致)
        angle    = -azimuth
        yaw      = np.pi / 2.0 - heading
        deltaYaw = yaw - ego_yaw
        x        = range_center * np.cos(angle)  # 纵向距离 (m)
        y        = range_center * np.sin(angle)  # 横向距离 (m)

        # 车道边界判断 (直道, ego 位于车道正中)
        y_left  =  HALF_LANE_WIDTH   # +1.8 m
        y_right = -HALF_LANE_WIDTH   # -1.8 m

        # 本车道过滤: y_right <= y <= y_left
        if y_right <= y <= y_left:
            # 相对纵向速度: ego 速度 - 前车纵向速度
            # v_rel > 0: ego 接近前车 (ego 比前车快)
            v_long = velocity * np.cos(deltaYaw)
            v_rel  = ego_speed - v_long

            # 相对距离 d_rel: 用 range_bbox (到车辆外壳的距离), 减去主车前悬长度
            d_rel = (range_bbox if range_bbox > 0 else x) - EGO_FRONT_OVERHANG

            ttc = calc_ttc(d_rel, v_rel)

            candidates.append({
                "id": id_,
                "d_rel": d_rel,
                "v_rel": v_rel,
                "ttc": ttc,
                "range_center": x,
                "velocity": velocity,
            })

    if not candidates:
        return None

    # 选择 TTC 最小的目标 (最危险)
    best = min(candidates, key=lambda x: x["ttc"])
    return best


# ============================================================================
# ModelOutput: 主循环
# ============================================================================

def ModelOutput(userData):
    sim_time = userData["time"]          # ms
    t_sec    = sim_time / 1000.0         # s

    # ========================================================================
    # 1. 读取主车状态
    # ========================================================================
    _, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed \
        = userData["ego_state"].readHeader()

    # ego_speed 单位确认: m/s (若是 km/h, 需转换)
    # PanoSim 中 ego speed 通常为 m/s, 这里做安全转换
    # 如果感觉速度偏高, 取消下面注释:
    # ego_speed = ego_speed / 3.6

    # ========================================================================
    # 2. 读取 GroundTruth Objects
    # ========================================================================
    _, num_objects = userData["gt_objects"].readHeader()
    objects = []
    for i in range(num_objects):
        obj = userData["gt_objects"].readBody(i)
        objects.append(obj)

    # ========================================================================
    # 3. 选择最危险目标
    # ========================================================================
    target = select_most_dangerous(objects, ego_speed, ego_yaw)

    if target is not None:
        d_rel  = target["d_rel"]
        v_rel  = target["v_rel"]
        ttc    = target["ttc"]
    else:
        d_rel  = 999.0
        v_rel  = 0.0
        ttc    = float("inf")

    # ========================================================================
    # 4. AEB 决策
    # ========================================================================
    state    = userData["aeb_state"]
    new_state, ttc_val = aeb_decision(ego_speed, d_rel, v_rel, userData["aeb_state"])
    userData["aeb_state"] = new_state

    # 期望减速度
    a_desired = calc_desired_accel(new_state, ego_speed)

    # 状态变化时打印
    if new_state != userData["last_aeb_state"]:
        state_names = ["None", "FCW", "PB1", "PB2", "FB"]
        print(f"[AEB] t={t_sec:.2f}s  TTC={ttc_val:.2f}s  State: "
              f"{state_names[new_state]}  a_des={a_desired:.1f} m/s²  "
              f"d_rel={d_rel:.1f}m  v_ego={ego_speed:.1f}m/s")
        userData["last_aeb_state"] = new_state

    # ========================================================================
    # 5. 写入 xDriver_accel_input 总线 (加速度输出)
    # ========================================================================
    valid = 1 if new_state >= 2 else 0
    userData["aeb_output"].writeHeader(sim_time, valid, a_desired)

    # ========================================================================
    # 6. 写入 Warning 总线 (FCW 告警)
    # ========================================================================
    if new_state >= 1:  # FCW 触发
        warning_text = b"FCW"
        userData["warning"].writeHeader(
            sim_time, 1, len(warning_text)
        )
        userData["warning"].getBus()[9:9+len(warning_text)] = warning_text

    # ========================================================================
    # 7. 实时曲线更新
    # ========================================================================
    if userData.get("enable_plot", False):
        _update_plot(userData, t_sec, ttc_val, a_desired, ego_speed, d_rel, new_state)


def _update_plot(userData, t_sec, ttc, a_dec, ego_spd, d_rel, state):
    """更新 matplotlib 实时曲线"""
    # 追加数据
    userData["time_hist"].append(t_sec)
    userData["ttc_hist"].append(ttc if ttc != float("inf") else 999.9)
    userData["dec_hist"].append(a_dec)
    userData["spd_hist"].append(ego_spd)
    userData["dist_hist"].append(d_rel)
    userData["state_hist"].append(state)

    # 滚动窗口, 最多保留 MAX_PLOT_POINTS
    max_pts = MAX_PLOT_POINTS
    if len(userData["time_hist"]) > max_pts:
        for key in ["time_hist", "ttc_hist", "dec_hist", "spd_hist",
                    "dist_hist", "state_hist"]:
            userData[key].pop(0)

    times = userData["time_hist"]
    t_max = max(times[-1] if times else 10.0, 10.0)
    t_min = max(0.0, t_max - 15.0)

    fig = userData["fig"]
    fig.canvas.restore_region(userData["bg"])

    ax_ttc, ax_dec, ax_spd, ax_state = userData["axes_list"]

    # TTC (显示时裁剪 >100 的异常值)
    line_ttc = userData["lines"]["ttc"]
    line_thr = userData["lines"]["thr"]
    ttc_disp = [min(v, 10.0) for v in userData["ttc_hist"]]
    line_ttc.set_data(times, ttc_disp)
    thr_vals = [TTC_THRESH_PB1] * len(times)
    line_thr.set_data(times, thr_vals)
    ax_ttc.set_xlim(t_min, t_max)
    ttc_max = max([v for v in userData["ttc_hist"] if v < 900] or [6.0])
    ax_ttc.set_ylim(0, max(6.0, ttc_max * 1.1))
    for artist in [line_ttc, line_thr]:
        ax_ttc.draw_artist(artist)
    ax_ttc.legend(loc="upper right", fontsize=8)

    # Deceleration
    line_dec = userData["lines"]["dec"]
    line_dec.set_data(times, userData["dec_hist"])
    ax_dec.set_xlim(t_min, t_max)
    ax_dec.set_ylim(min(-10.0, min(userData["dec_hist"]) - 1.0), 2.0)
    ax_dec.axhline(0, color="gray", lw=0.8)
    ax_dec.draw_artist(line_dec)
    ax_dec.legend(loc="upper right", fontsize=8)

    # Ego Speed + Relative Distance (显示时裁剪 d_rel>200 的异常值)
    line_spd   = userData["lines"]["spd"]
    line_rdist = userData["lines"]["rdist"]
    dist_disp = [min(v, 200.0) for v in userData["dist_hist"]]
    line_spd.set_data(times, userData["spd_hist"])
    line_rdist.set_data(times, dist_disp)
    ax_spd.set_xlim(t_min, t_max)
    spd_max = max(userData["spd_hist"])
    dist_max = max([v for v in userData["dist_hist"] if v < 900] or [40.0])
    ax_spd.set_ylim(0, max(40.0, spd_max, dist_max) * 1.1)
    for artist in [line_spd, line_rdist]:
        ax_spd.draw_artist(artist)
    ax_spd.legend(loc="upper right", fontsize=8)

    # AEB State
    line_state = userData["lines"]["state"]
    line_state.set_data(times, userData["state_hist"])
    ax_state.set_xlim(t_min, t_max)
    ax_state.set_ylim(-0.5, 4.5)
    ax_state.draw_artist(line_state)

    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()


# ============================================================================
# ModelTerminate: 清理
# ============================================================================

def ModelTerminate(userData):
    if userData.get("enable_plot", False):
        plt.ioff()
        plt.close("AEB Metrics")
    print("[AEB] Plugin terminated.")
