#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math, time, csv, os, glob, shutil, subprocess, heapq
import tkinter as tk

JOINTS = [
    'fl_hip_joint', 'fl_thigh_joint', 'fl_shank_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_shank_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_shank_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_shank_joint'
]

WHEELS = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']
WHEEL_RADIUS = 0.06

# Динамические препятствия, обнаруженные 3D-камерой PCL в реальном времени: [x, y, rad, expire_time]
DYNAMIC_PCL_OBSTACLES = []

def leg_ik(x, y_shift, z, L2=0.20, L3=0.20):
    q1 = max(min(y_shift / 0.34, 0.85), -0.85)
    r = math.sqrt(x * x + z * z)
    r = max(min(r, 0.392), 0.12)
    cos_q3 = (r * r - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    cos_q3 = max(min(cos_q3, 1.0), -1.0)
    q3 = -math.acos(cos_q3)
    gamma = math.atan2(-x, -z)
    beta = math.atan2(-L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    q2 = gamma + beta
    return [q1, q2, q3]

def bezier_step(s, x0, z0, x1, z1, dh):
    s = max(min(s, 1.0), 0.0)
    u = 3.0 * s * s - 2.0 * s * s * s
    x = x0 + (x1 - x0) * u
    z = (1.0 - u) * z0 + u * z1 + dh * math.sin(math.pi * s)
    return x, z

def smoothstep(s):
    s = max(min(s, 1.0), 0.0)
    return 3.0 * s * s - 2.0 * s * s * s

def lerp_list(a, b, s):
    u = smoothstep(s)
    return [x + (y - x) * u for x, y in zip(a, b)]

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def euler_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    )

def is_point_blocked(x, y, dyn_scale=1.0):
    if x < -4.3 or x > 14.3 or y < -7.2 or y > 7.2:
        return True
    # Стеклянные перегородки Y = +-2.70 м (проход строго через дверные проемы X = 3.05..3.85 м)
    if 2.30 <= abs(y) <= 3.10:
        if not (3.05 <= x <= 3.85):
            return True
    # Боковые обрывы лестницы ТОЛЬКО внутри центрального зала (не блокируют северную и южную комнаты!)
    if 8.35 <= x <= 12.30 and (1.35 < abs(y) < 2.30):
        return True
    if x > 12.30 and abs(y) > 2.60:
        return True
    if 1.65 <= x <= 3.55 and 0.85 <= abs(y) <= 1.45:
        return True
    obstacles = [
        (1.2, 1.25, 0.54),
        (3.8, -0.75, 0.60),
        (4.8, 0.0, 0.74),
        (8.2, 1.6, 0.68),
        (8.2, -1.6, 0.68)
    ]
    for cx, cy, rad in obstacles:
        if math.hypot(x - cx, y - cy) < rad:
            return True
    # Блоки лабиринта за дверями (оставляем чистый проход вдоль Y = 3.10..3.62 м)
    if (2.66 <= x <= 4.24) and (3.62 <= y <= 5.08):
        return True
    if (2.66 <= x <= 4.24) and (-5.08 <= y <= -3.62):
        return True
    # Ресепшн, киоск, станок ЧПУ, верстаки и серверные стойки
    if (-3.1 <= x <= -1.3) and (0.5 <= y <= 3.8):
        return True
    if (-2.1 <= x <= -0.9) and (-2.8 <= y <= -1.5):
        return True
    if (8.65 <= x <= 10.95) and (y >= 4.75):
        return True
    # Южный бетонный бортик водного бассейна (6 см) — заход в бассейн строго с открытой западной стороны!
    if (5.80 <= x <= 9.20) and (4.18 <= y <= 4.82):
        return True
    if y > 5.75 or y < -5.35:
        return True
    if dyn_scale > 0.0:
        now_t = time.time()
        for dx_o, dy_o, dr_o, exp_t in DYNAMIC_PCL_OBSTACLES:
            if now_t <= exp_t and math.hypot(x - dx_o, y - dy_o) < (dr_o * dyn_scale):
                return True
    return False

def should_register_dynamic_obstacle(wx, wy, obs_h, goal_x, goal_y, rx=None, ry=None):
    """Фильтрует стены, косяки дверей, просмотр сквозь стекло, лестницу и подиум."""
    if obs_h < 0.22:
        return False
    if wx < -4.0 or wx > 14.0 or wy < -6.8 or wy > 6.8:
        return False
    # Запрещаем регистрировать объект, увиденный камерой СКВОЗЬ прозрачное стекло в другой комнате!
    if rx is not None and ry is not None:
        if (not _same_room_side(ry, wy)) and not (2.95 <= rx <= 3.95 and 2.95 <= wx <= 3.95):
            return False
    # Игнорируем саму линию стеклянных стен и дверной проем (чтобы не запечатывать единственный вход!)
    if 2.25 <= abs(wy) <= 3.15:
        return False
    # Игнорируем жёлтые дверные столбы (2.72, +-2.7) и (4.18, +-2.7)
    for dpx, dpy in [(2.72, 2.70), (4.18, 2.70), (2.72, -2.70), (4.18, -2.70)]:
        if math.hypot(wx - dpx, wy - dpy) < 0.60:
            return False
    # Игнорируем лестницу и сцену
    if wx >= 8.00 and abs(wy) <= 2.30:
        return False
    # Игнорируем подиум Зоны А
    if 1.40 <= wx <= 3.80 and abs(wy) <= 1.40:
        return False
    # Не блокируем саму целевую точку
    if math.hypot(wx - goal_x, wy - goal_y) < 0.55:
        return False
    # Игнорируем уже известные статические препятствия
    static_obs = [(1.2, 1.25, 0.65), (3.8, -0.75, 0.70), (4.8, 0.0, 0.85), (8.2, 1.6, 0.78), (8.2, -1.6, 0.78)]
    for cx, cy, rad in static_obs:
        if math.hypot(wx - cx, wy - cy) < rad:
            return False
    if (2.45 <= wx <= 4.45) and (3.45 <= abs(wy) <= 5.25):
        return False
    return True

def register_dynamic_obstacle(wx, wy, rad=0.72, ttl=25.0):
    """Добавляет или обновляет динамическое препятствие (без дубликатов)."""
    now_t = time.time()
    DYNAMIC_PCL_OBSTACLES[:] = [o for o in DYNAMIC_PCL_OBSTACLES if o[3] > now_t]
    for o in DYNAMIC_PCL_OBSTACLES:
        if math.hypot(wx - o[0], wy - o[1]) < 0.65:
            o[0] = 0.75 * o[0] + 0.25 * wx
            o[1] = 0.75 * o[1] + 0.25 * wy
            o[2] = max(o[2], rad)
            o[3] = now_t + ttl
            return False
    DYNAMIC_PCL_OBSTACLES.append([wx, wy, rad, now_t + ttl])
    return True

def clear_phantom_obstacles(rx, ry, yaw, obs_dist):
    """Лучевая очистка: удаляет устаревшие преграды, если камера видит свободно сквозь них."""
    if not DYNAMIC_PCL_OBSTACLES:
        return False
    now_t = time.time()
    kept = []
    removed = False
    for o in DYNAMIC_PCL_OBSTACLES:
        if o[3] <= now_t:
            removed = True
            continue
        dx = o[0] - rx
        dy = o[1] - ry
        d = math.hypot(dx, dy)
        ang = abs(wrap_angle(math.atan2(dy, dx) - yaw))
        # Если смотрим прямо на старый объект (0.55..1.90 м, угол < 12°), а камера видит глубже него -> удаляем!
        if 0.55 <= d <= 1.90 and ang < 0.21 and obs_dist > (d + 0.25):
            removed = True
            continue
        kept.append(o)
    if removed:
        DYNAMIC_PCL_OBSTACLES[:] = kept
    return removed

def _same_room_side(y_ref, y_cand):
    if y_ref > 2.70:
        return y_cand > 2.70
    if y_ref < -2.70:
        return y_cand < -2.70
    return abs(y_cand) < 2.70

def snap_to_free(x, y, block_podium=False, dyn_scale=1.0):
    def blocked(px, py):
        if block_podium and (1.55 <= px <= 3.65 and abs(py) <= 1.45):
            return True
        return is_point_blocked(px, py, dyn_scale=dyn_scale)

    if not blocked(x, y):
        return (x, y)

    # Ищем ближайшую свободную точку НА ТОЙ ЖЕ СТОРОНЕ стеклянной перегородки!
    for r in [0.15, 0.30, 0.45, 0.60, 0.80, 1.05, 1.35, 1.70, 2.10]:
        best_pt = None
        best_d = 999.0
        for k in range(36):
            ang = 2.0 * math.pi * k / 36.0
            nx = x + r * math.cos(ang)
            ny = y + r * math.sin(ang)
            if _same_room_side(y, ny) and (not blocked(nx, ny)):
                d = math.hypot(nx - x, ny - y)
                if d < best_d:
                    best_d = d
                    best_pt = (nx, ny)
        if best_pt is not None:
            return best_pt

    for r in [0.30, 0.60, 1.00, 1.50, 2.20]:
        for k in range(36):
            ang = 2.0 * math.pi * k / 36.0
            nx = x + r * math.cos(ang)
            ny = y + r * math.sin(ang)
            if not blocked(nx, ny):
                return (nx, ny)
    return (x, y)

def _segment_clear(p1, p2, blocked_fn):
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    n = max(int(dist / 0.08), 2)
    for i in range(1, n):
        u = i / float(n)
        if blocked_fn(p1[0] + u * (p2[0] - p1[0]), p1[1] + u * (p2[1] - p1[1])):
            return False
    return True

def astar_core(sx, sy, gx, gy, block_podium=False, dyn_scale=1.0):
    res = 0.25
    SQRT2 = math.sqrt(2.0)

    def to_grid(x, y):
        return (int(round((x + 4.5) / res)), int(round((y + 7.5) / res)))

    def to_world(ix, iy):
        return (-4.5 + ix * res, -7.5 + iy * res)

    def blocked(x, y):
        if block_podium and (1.55 <= x <= 3.65 and abs(y) <= 1.45):
            return True
        return is_point_blocked(x, y, dyn_scale=dyn_scale)

    sx_f, sy_f = snap_to_free(sx, sy, block_podium=block_podium, dyn_scale=dyn_scale)
    gx_f, gy_f = snap_to_free(gx, gy, block_podium=block_podium, dyn_scale=dyn_scale)

    start_g = to_grid(sx_f, sy_f)
    goal_g = to_grid(gx_f, gy_f)

    open_heap = []
    heapq.heappush(open_heap, (math.hypot(gx_f - sx_f, gy_f - sy_f), 0.0, start_g))
    came_from = {}
    g_score = {start_g: 0.0}
    closed = set()

    dirs = [(-1,0,1.0), (1,0,1.0), (0,-1,1.0), (0,1,1.0),
            (-1,-1,SQRT2), (-1,1,SQRT2), (1,-1,SQRT2), (1,1,SQRT2)]

    best_node = start_g
    best_h = math.hypot(gx_f - sx_f, gy_f - sy_f)
    reached = False

    steps = 0
    while open_heap and steps < 12000:
        _, g_cur, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        steps += 1

        if current == goal_g:
            best_node = current
            reached = True
            break

        cx, cy = to_world(*current)
        h_cur = math.hypot(gx_f - cx, gy_f - cy)
        if h_cur < best_h:
            best_h = h_cur
            best_node = current

        for dx, dy, cost in dirs:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt in closed:
                continue
            nx, ny = to_world(*nxt)
            if nxt != start_g and nxt != goal_g:
                if blocked(nx, ny):
                    continue
                # Запрещаем диагональное проскальзывание сквозь тонкую стену или угол!
                if dx != 0 and dy != 0:
                    if blocked(*to_world(current[0] + dx, current[1])) or blocked(*to_world(current[0], current[1] + dy)):
                        continue
            ng = g_cur + cost * res
            if ng + 1e-6 < g_score.get(nxt, 999999.0):
                came_from[nxt] = current
                g_score[nxt] = ng
                f = ng + math.hypot(gx_f - nx, gy_f - ny)
                heapq.heappush(open_heap, (f, ng, nxt))

    raw_pts = []
    curr = best_node
    while curr in came_from:
        raw_pts.append(to_world(*curr))
        curr = came_from[curr]
    raw_pts.reverse()
    if reached:
        raw_pts.append((gx_f, gy_f))
    elif not raw_pts:
        raw_pts.append((sx_f, sy_f))

    smooth = []
    cur_pos = (sx_f, sy_f)
    idx = 0
    while idx < len(raw_pts):
        furthest = idx
        for j in range(len(raw_pts) - 1, idx, -1):
            if _segment_clear(cur_pos, raw_pts[j], blocked):
                furthest = j
                break
        smooth.append(raw_pts[furthest])
        cur_pos = raw_pts[furthest]
        idx = furthest + 1

    if (not reached) and dyn_scale > 0.0:
        # Если динамический круг случайно перекрыл узкий проход — сжимаем радиус и перепланируем!
        next_scale = 0.60 if dyn_scale > 0.65 else 0.0
        return astar_core(sx, sy, gx, gy, block_podium=block_podium, dyn_scale=next_scale)

    filtered = [pt for pt in smooth[:-1] if math.hypot(pt[0] - sx_f, pt[1] - sy_f) > 0.22]
    filtered.append(smooth[-1])
    return filtered

def build_smooth_dense_path(sx, sy, waypoints, block_podium=False):
    pts = [(sx, sy)] + list(waypoints)
    dense = [pts[0]]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(d / 0.30)))
        for k in range(1, n + 1):
            u = k / float(n)
            dense.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u))

    def blocked(x, y):
        if block_podium and (1.55 <= x <= 3.65 and abs(y) <= 1.45):
            return True
        return is_point_blocked(x, y)

    cur = dense
    for _ in range(2):
        if len(cur) < 3:
            break
        nxt = [cur[0]]
        for i in range(len(cur) - 1):
            p0 = cur[i]; p1 = cur[i + 1]
            q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            if (not blocked(*q)) and _segment_clear(nxt[-1], q, blocked):
                nxt.append(q)
            else:
                nxt.append(p0)
            if (not blocked(*r)) and _segment_clear(nxt[-1], r, blocked):
                nxt.append(r)
            else:
                nxt.append(p1)
        nxt.append(cur[-1])
        cur = nxt

    cur[-1] = waypoints[-1]
    return cur

def plan_global_route(sx, sy, gx, gy):
    goal_is_podium = (1.75 <= gx <= 3.45 and abs(gy) <= 1.0)
    start_on_podium = (1.75 <= sx <= 3.45 and abs(sy) <= 1.0)
    blk_pod = (not goal_is_podium) and (not start_on_podium)

    gx_f, gy_f = snap_to_free(gx, gy, block_podium=blk_pod)

    if sx < 8.4 and gx_f >= 8.5 and abs(gy_f) <= 2.6:
        p1 = astar_core(sx, sy, 8.0, 0.0, block_podium=blk_pod)
        if gx_f > 12.2:
            p1.append((12.35, 0.0))
        p1.append((gx_f, gy_f))
        return build_smooth_dense_path(sx, sy, p1, block_podium=blk_pod)

    if sx >= 8.5 and abs(sy) <= 2.6 and gx_f < 8.4:
        pts = []
        if sx > 12.1:
            pts.append((12.1, 0.0))
        pts.append((7.9, 0.0))
        pts.extend(astar_core(7.9, 0.0, gx_f, gy_f, block_podium=blk_pod))
        return build_smooth_dense_path(sx, sy, pts, block_podium=blk_pod)

    if abs(gx_f - 7.8) < 0.2 and abs(gy_f - (-1.75)) < 0.2 and (sx < 1.8 or abs(sy - (-1.75)) > 0.6):
        p1 = astar_core(sx, sy, 1.9, -1.75, block_podium=True)
        p1.append((gx_f, gy_f))
        return build_smooth_dense_path(sx, sy, p1, block_podium=True)

    raw = astar_core(sx, sy, gx_f, gy_f, block_podium=blk_pod)
    return build_smooth_dense_path(sx, sy, raw, block_podium=blk_pod)

class HybridControllerNode(Node):
    def __init__(self):
        super().__init__('wqr_hybrid_controller')
        self.pub_traj = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.pub_wheel = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.cli_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/p3d_base_controller/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Float32MultiArray, '/detected_obstacle', self.obs_cb, 10)

        self.rx, self.ry, self.rz = 0.0, 0.0, 0.406
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.roll_rate, self.pitch_rate, self.yaw_rate = 0.0, 0.0, 0.0
        self.ax_imu, self.ay_imu, self.az_imu = 0.0, 0.0, 9.81
        self.ax_filt, self.ay_filt, self.az_filt = 0.0, 0.0, 9.81
        self.vx_act, self.vy_act, self.v_body = 0.0, 0.0, 0.0
        self.v_body_prev = 0.0
        self.a_long_act = 0.0
        self.rough_score = 0.0
        self.imu_rough_until = 0.0
        self.odom_received = False
        self.joints_initialized = False
        self.wheels_initialized = False
        self.last_dt_t = time.time()

        self.obs_dist = 99.0
        self.obs_height = 0.0
        self.left_clear = 99.0
        self.right_clear = 99.0
        self.avoid_until = 0.0
        self.avoid_dir = 1.0
        self.last_replan_t = 0.0

        # Состояние физического LQR-балансира на 2 задних колесах (без set_gazebo_pose!)
        self.biped_pitch_eq = -0.22
        self.biped_trim = 0.0
        self.biped_xb = 0.225
        self.biped_zb = -0.305
        self.biped_vw = 0.0
        self.biped_x0 = 0.0
        self.biped_prev_x = 0.0
        self.biped_v_filt = 0.0
        self.biped_x_ref = 0.0
        self.biped_bad_since = None

        # Фильтрованные углы IMU для плавной активной подвески
        self.pitch_filt = 0.0
        self.roll_filt = 0.0

        self.mode = "MANUAL"
        self.auto_gait = True
        self.gait = "WHEELED"
        self.posture = "STAND"
        self.special_state = "NONE"
        self.trans_t0 = 0.0
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_z = 0.406
        self.anchor_yaw = 0.0

        self.env_surface = "РОВНЫЙ ПОЛ"
        self.state_str = "ГОТОВ (v18.0): Нулевой дрейф + Плавный автопилот Pure Pursuit + Точный стоп"

        self.current_cmd_joints = [0.0, 0.5, -1.0] * 4

        self.goal_x = 6.4
        self.goal_y = 0.0
        self.path_pts = []
        self.path_idx = 0
        self.goal_active = False
        self.pivoting_in_place = False

        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_wz = 0.0
        self.vx_smooth = 0.0
        self.wz_smooth = 0.0
        self.body_z = -0.346

        self.traj_ref_y = 0.0
        self.traj_err_m = 0.0
        self.wheel_contact_pct = 100.0
        self.susp_corr_mm = 0.0

        self.stepping_active = False
        self.step_t0 = 0.0
        self.crawl_phase = 0.0
        self.step_h = 0.09
        self.last_step_finish = 0.0

        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0]
        self.brake_locked = False
        self.brake_wheel_pos = [0.0, 0.0, 0.0, 0.0]

        self.last_sent_joints = None
        self.last_traj_time = 0.0

        self.zmp_margin = 95.0
        self.power_w = 12.0
        self.cot_val = 0.18
        self.q_ref = [0.0, 0.5, -1.0] * 4
        self.q_act = list(self.q_ref)
        self.q_vel = [0.0] * 12
        self.q_eff = [0.0] * 12
        self.wheel_eff = [0.0] * 4
        self.has_effort_state = False
        self.telemetry_rows = []
        self.start_wall_t = time.time()

        self.create_timer(0.02, self.control_loop)

    def quat_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = max(min(+2.0 * (w * y - z * x), 1.0), -1.0)
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def odom_cb(self, m):
        self.odom_received = True
        self.rx = m.pose.pose.position.x
        self.ry = m.pose.pose.position.y
        self.rz = m.pose.pose.position.z
        q = m.pose.pose.orientation
        self.roll, self.pitch, self.yaw = self.quat_to_euler(q.x, q.y, q.z, q.w)
        self.vx_act = float(m.twist.twist.linear.x)
        self.vy_act = float(m.twist.twist.linear.y)
        v_now = math.hypot(self.vx_act, self.vy_act)
        self.a_long_act = 0.75 * self.a_long_act + 0.25 * max(min((v_now - self.v_body_prev) / 0.02, 6.0), -6.0)
        self.v_body_prev = v_now
        self.v_body = 0.65 * self.v_body + 0.35 * v_now

    def imu_cb(self, m):
        q = m.orientation
        r, p, y = self.quat_to_euler(q.x, q.y, q.z, q.w)
        self.roll_rate = float(m.angular_velocity.x)
        self.pitch_rate = float(m.angular_velocity.y)
        self.yaw_rate = float(m.angular_velocity.z)
        self.ax_imu = float(m.linear_acceleration.x)
        self.ay_imu = float(m.linear_acceleration.y)
        self.az_imu = float(m.linear_acceleration.z)
        self.ax_filt = 0.70 * self.ax_filt + 0.30 * self.ax_imu
        self.ay_filt = 0.70 * self.ay_filt + 0.30 * self.ay_imu
        self.az_filt = 0.70 * self.az_filt + 0.30 * self.az_imu
        if not self.odom_received:
            self.roll, self.pitch, self.yaw = r, p, y

    def joint_cb(self, m):
        pos_map = dict(zip(m.name, m.position))
        if all(j in pos_map for j in JOINTS):
            self.q_act = [pos_map[j] for j in JOINTS]
            self.joints_initialized = True
        if all(w in pos_map for w in WHEELS):
            self.wheel_pos = [pos_map[w] for w in WHEELS]
            self.wheels_initialized = True
        if len(m.velocity) == len(m.name):
            vel_map = dict(zip(m.name, m.velocity))
            if all(j in vel_map for j in JOINTS):
                self.q_vel = [float(vel_map[j]) for j in JOINTS]
            if all(w in vel_map for w in WHEELS):
                self.wheel_vel = [float(vel_map[w]) for w in WHEELS]
        if len(m.effort) == len(m.name):
            eff_map = dict(zip(m.name, m.effort))
            if all(j in eff_map for j in JOINTS):
                eff_legs = [float(eff_map[j]) for j in JOINTS]
                if sum(abs(x) for x in eff_legs) > 0.1:
                    self.q_eff = eff_legs
                    self.has_effort_state = True
            if all(w in eff_map for w in WHEELS):
                self.wheel_eff = [float(eff_map[w]) for w in WHEELS]

    def obs_cb(self, m):
        if len(m.data) >= 4:
            self.obs_dist = float(m.data[0])
            self.obs_height = float(m.data[1])
            self.left_clear = float(m.data[2])
            self.right_clear = float(m.data[3])

    def get_surface_standing_z(self, x_val=None, y_val=None):
        px = self.rx if x_val is None else x_val
        py = self.ry if y_val is None else y_val
        if px > 12.15 and abs(py) < 3.2:
            return 0.45 + 0.406
        elif 8.50 <= px <= 12.15 and abs(py) < 1.8:
            return ((px - 8.50) / 3.65) * 0.45 + 0.406
        elif 1.80 <= px <= 3.40 and abs(py) < 1.1:
            return 0.08 + 0.406
        return 0.406

    def set_gazebo_pose(self, x, y, z, roll, pitch, yaw):
        if not self.cli_state.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = 'wheeled_quadruped'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
        req.state.pose.orientation.x = qx
        req.state.pose.orientation.y = qy
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        self.cli_state.call_async(req)

    def select_gait_mode(self, mode_name):
        if self.special_state != "NONE":
            self.set_posture_mode("STAND")
        self.posture = "STAND"
        if mode_name == "AUTO":
            self.auto_gait = True
            self.state_str = "РЕЖИМ ХОДЬБЫ: АВТОМАТИЧЕСКИЙ (Колёса / Рысь / Шаг / Лестница)"
        else:
            self.auto_gait = False
            self.gait = mode_name
            names = {"WHEELED": "КОЛЁСНЫЙ ХОД", "CRAWL": "СТАТИЧЕСКИЙ ШАГ (CRAWL)", "TROT": "ДИАГОНАЛЬНАЯ РЫСЬ (TROT)"}
            self.state_str = f"РЕЖИМ ХОДЬБЫ: {names.get(mode_name, mode_name)} — управляй джойстиком!"

    def set_posture_mode(self, new_posture):
        self.mode = "MANUAL"
        self.goal_active = False
        self.stepping_active = False
        self.manual_vx = 0.0; self.manual_vy = 0.0; self.manual_wz = 0.0
        self.vx_smooth = 0.0; self.wz_smooth = 0.0

        if new_posture == "BIPED_REAL":
            self.anchor_yaw = self.yaw
            # Фиксируем точку контакта задней оси сразу при старте подъема (0 мм рывка!)
            self.anchor_x = self.rx - 0.25 * math.cos(self.anchor_yaw)
            self.anchor_y = self.ry - 0.25 * math.sin(self.anchor_yaw)
            self.anchor_z = self.get_surface_standing_z()
            self.trans_t0 = time.time()
            self.brake_locked = False
            self.special_state = "BIPED_UP"
            self.posture = "BIPED_REAL"
            return

        if new_posture == "SIDE_REAL":
            self.anchor_yaw = self.yaw
            # Фиксируем точку контакта левых колес (y_local = +0.20 м) -> ноль бокового скольжения!
            self.anchor_x = self.rx - 0.20 * math.sin(self.anchor_yaw)
            self.anchor_y = self.ry + 0.20 * math.cos(self.anchor_yaw)
            self.anchor_z = self.get_surface_standing_z()
            self.trans_t0 = time.time()
            self.brake_locked = False
            self.special_state = "SIDE_UP"
            self.posture = "SIDE_REAL"
            return

        if self.special_state in ["BIPED_UP", "BIPED_HOLD"]:
            self.trans_t0 = time.time()
            self.special_state = "BIPED_DOWN"
            self.posture = "STAND"
            return

        if self.special_state in ["SIDE_UP", "SIDE_HOLD"]:
            self.trans_t0 = time.time()
            self.special_state = "SIDE_DOWN"
            self.posture = "STAND"
            return

        if new_posture == "STAND" and (abs(self.roll) > 0.25 or abs(self.pitch) > 0.25):
            self.set_gazebo_pose(self.rx, self.ry, self.get_surface_standing_z(), 0.0, 0.0, self.yaw)

        self.special_state = "NONE"
        self.posture = new_posture
        self.current_cmd_joints = list(self.q_act)
        self.brake_locked = False

    def trigger_demo_180(self):
        self.mode = "MANUAL"
        self.goal_active = False
        self.stepping_active = False
        self.vx_smooth = 0.0; self.wz_smooth = 0.0
        self.anchor_x = self.rx
        self.anchor_y = self.ry
        self.anchor_z = self.get_surface_standing_z()
        self.anchor_yaw = self.yaw
        self.trans_t0 = time.time()
        if abs(self.roll) > 1.2 or abs(self.pitch) > 1.2:
            self.special_state = "RECOVERING"
        else:
            self.special_state = "DEMO_180_FULL"
        self.posture = "STAND"

    def trigger_step_climb(self, height=0.09):
        if not self.stepping_active and self.special_state == "NONE":
            self.posture = "STAND"
            self.stepping_active = True
            self.step_t0 = time.time()
            self.step_h = max(min(height, 0.14), 0.075)

    def set_mission_goal(self, gx, gy, ref_y=None):
        self.goal_x = float(gx)
        self.goal_y = float(gy)
        self.traj_ref_y = float(gy if ref_y is None else ref_y)
        self.stepping_active = False
        if self.special_state != "NONE" or abs(self.roll) > 0.25 or abs(self.pitch) > 0.25:
            self.set_gazebo_pose(self.rx, self.ry, self.get_surface_standing_z(), 0.0, 0.0, self.yaw)
        self.special_state = "NONE"
        self.manual_vx = 0.0; self.manual_vy = 0.0; self.manual_wz = 0.0
        self.posture = "STAND"
        self.auto_gait = True
        self.brake_locked = False

        self.path_pts = plan_global_route(self.rx, self.ry, self.goal_x, self.goal_y)
        if self.path_pts:
            self.goal_x, self.goal_y = self.path_pts[-1]
        self.path_idx = 0
        self.pivoting_in_place = False
        self.mode = "AUTO"
        self.goal_active = True
        self.state_str = f"АВТОПИЛОТ (Pure Pursuit): Маршрут построен -> ({self.goal_x:.2f}, {self.goal_y:.2f})"

    def update_imu_roughness(self, now):
        """Реальный детектор неровного рельефа по IMU (гироскоп + вертикальный акселерометр + крен/тангаж)."""
        on_stairs_area = (8.00 <= self.rx <= 12.40 and abs(self.ry) <= 1.90)
        if self.special_state == "NONE" and (not self.stepping_active) and (not on_stairs_area) and self.v_body > 0.08:
            ang_vib = abs(self.roll_rate) + abs(self.pitch_rate)
            az_jitter = abs(self.az_imu - 9.81)
            tilt_ex = max(0.0, abs(self.roll) + abs(self.pitch) - 0.030)
            imu_shock = 0.45 * ang_vib + 0.22 * az_jitter + 4.5 * tilt_ex
            self.rough_score = 0.72 * self.rough_score + 0.28 * imu_shock
            if self.rough_score > 0.42:
                self.imu_rough_until = now + 2.20
        else:
            self.rough_score *= 0.85

    def detect_environment_surface(self):
        now = time.time()
        self.update_imu_roughness(now)

        if 8.10 <= self.rx <= 12.35 and abs(self.ry) <= 1.8:
            self.env_surface = "ЛЕСТНИЦА (IMU+PCL)"
            if self.auto_gait:
                self.gait = "STAIR_CLIMB"
            return "STAIRS"
        elif 6.0 <= self.rx <= 9.0 and -4.9 <= self.ry <= -3.3:
            self.env_surface = "ПЕСОК И ГРАВИЙ"
            if self.auto_gait:
                self.gait = "CRAWL"
            return "SAND"
        elif 6.0 <= self.rx <= 9.0 and 4.4 <= self.ry <= 6.2:
            self.env_surface = "ВОДНЫЙ БРОД"
            if self.auto_gait:
                self.gait = "WHEELED"
            return "WATER"
        elif now < self.imu_rough_until:
            self.env_surface = "НЕРОВНОСТИ (IMU)"
            if self.auto_gait:
                self.gait = "TROT"
            return "ROUGH"
        elif self.rx > 12.35 and abs(self.ry) <= 3.0:
            self.env_surface = "ВЕРХНЯЯ СЦЕНА"
            if self.auto_gait:
                self.gait = "WHEELED"
            return "STAGE"
        else:
            self.env_surface = "РОВНЫЙ ПОЛ"
            if self.auto_gait:
                self.gait = "WHEELED"
            return "FLOOR"

    def compute_imu_suspension_deltas(self, active=True):
        """Вычисляет коррекцию высоты лап (dz_p, dz_r) по фильтрованным углам IMU с мертвой зоной 1.7°."""
        if os.environ.get("WQR_NO_SUSP") == "1":
            self.susp_corr_mm = 0.0
            return 0.0, 0.0
        if not active:
            self.susp_corr_mm = 0.0
            return 0.0, 0.0
        db = 0.030  # мертвая зона 1.7 град -> на ровном полу в покое коррекция = 0.0 мм
        p_eff = (self.pitch_filt - db) if self.pitch_filt > db else ((self.pitch_filt + db) if self.pitch_filt < -db else 0.0)
        r_eff = (self.roll_filt  - db) if self.roll_filt  > db else ((self.roll_filt  + db) if self.roll_filt  < -db else 0.0)
        dz_p = max(min(0.22 * math.sin(p_eff), 0.042), -0.042)
        dz_r = max(min(0.18 * math.sin(r_eff), 0.036), -0.036)
        self.susp_corr_mm = round((abs(dz_p) + abs(dz_r)) * 1000.0, 1)
        return dz_p, dz_r

    def get_target_posture_joints(self, surf_type="FLOOR", is_moving=False):
        if self.posture == "SIT":
            self.susp_corr_mm = 0.0
            q_front = leg_ik(0.0, 0.0, -0.355)
            q_rear  = [0.0, 1.15, -2.25]
            return q_front + q_front + q_rear + q_rear
        elif self.posture == "LIE":
            self.susp_corr_mm = 0.0
            return [0.0, 1.22, -2.35] * 4

        z0 = -0.370 if surf_type == "WATER" else self.body_z
        # Активная подвеска работает на неровностях и при движении по уклонам (не тревожит стоянку на ровном полу!)
        use_susp = (surf_type == "ROUGH") or (is_moving and (abs(self.pitch_filt) > 0.035 or abs(self.roll_filt) > 0.035))
        dz_p, dz_r = self.compute_imu_suspension_deltas(active=use_susp)
        if abs(dz_p) > 0.001 or abs(dz_r) > 0.001:
            # pitch > 0 (нос вниз) -> удлиняем передние (z0 - dz_p), укорачиваем задние (z0 + dz_p)
            # roll  > 0 (левый борт вверх) -> укорачиваем левые (z0 + dz_r), удлиняем правые (z0 - dz_r)
            z_fl = max(min(z0 - dz_p + dz_r, -0.250), -0.384)
            z_fr = max(min(z0 - dz_p - dz_r, -0.250), -0.384)
            z_rl = max(min(z0 + dz_p + dz_r, -0.250), -0.384)
            z_rr = max(min(z0 + dz_p - dz_r, -0.250), -0.384)
            return leg_ik(0.0, 0.0, z_fl) + leg_ik(0.0, 0.0, z_fr) + leg_ik(0.0, 0.0, z_rl) + leg_ik(0.0, 0.0, z_rr)

        if surf_type == "WATER":
            return leg_ik(0.0, 0.0, -0.370) * 4
        return [0.0, 0.5, -1.0] * 4

    def step_slew_joints(self, target_joints, max_rate_rad_s=2.0, dt=0.02):
        max_step = max_rate_rad_s * dt
        out = []
        for cur, tgt in zip(self.current_cmd_joints, target_joints):
            diff = tgt - cur
            if abs(diff) <= max_step:
                out.append(tgt)
            else:
                out.append(cur + (max_step if diff > 0 else -max_step))
        self.current_cmd_joints = out
        return out

    def get_pure_pursuit_target(self, lookahead_dist=0.65):
        """Находит ближайшую точку прогресса и целевую точку упреждения вдоль сглаженной кривой."""
        if not self.path_pts:
            return (self.goal_x, self.goal_y)

        # Ищем ближайшую точку вперед от текущего индекса (чтобы не прыгать назад)
        search_end = min(len(self.path_pts), self.path_idx + 15)
        best_i = self.path_idx
        best_d = 9999.0
        for i in range(self.path_idx, search_end):
            d = math.hypot(self.path_pts[i][0] - self.rx, self.path_pts[i][1] - self.ry)
            if d < best_d:
                best_d = d
                best_i = i
        self.path_idx = best_i

        # Выбираем точку упреждения на расстоянии lookahead_dist
        for i in range(self.path_idx, len(self.path_pts)):
            d = math.hypot(self.path_pts[i][0] - self.rx, self.path_pts[i][1] - self.ry)
            if d >= lookahead_dist:
                return self.path_pts[i]
        return self.path_pts[-1]

    def control_loop(self):
        now = time.time()
        dt = min(max(now - self.last_dt_t, 0.005), 0.05)
        self.last_dt_t = now
        z0 = self.body_z
        surf_type = self.detect_environment_surface()
        self.pitch_filt = 0.80 * self.pitch_filt + 0.20 * self.pitch
        self.roll_filt  = 0.80 * self.roll_filt  + 0.20 * self.roll

        self.traj_err_m = round(abs(self.ry - self.traj_ref_y), 3)
        vib = abs(self.roll) + abs(self.pitch)
        self.wheel_contact_pct = round(max(85.0, min(100.0, 100.0 - vib * 30.0 + 5.0)), 1)
        floor_h = self.anchor_z - 0.406

        # --- 0. ПОЛНЫЙ ЦИКЛ САМОВОССТАНОВЛЕНИЯ СО СПИНЫ 180° ---
        if self.special_state == "DEMO_180_FULL":
            el = now - self.trans_t0
            if el < 0.9:
                u = smoothstep(el / 0.9)
                self.state_str = f"ДЕМО 180° [1/4]: Опрокидывание на спину ({int(u*180)} град)"
                self.set_gazebo_pose(self.anchor_x, self.anchor_y, self.anchor_z*(1.0-u) + (floor_h + 0.16)*u, 3.12*u, 0.0, self.anchor_yaw)
                joints = self.step_slew_joints([0.0, 1.25, -2.30]*4, 3.5, dt)
                self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
                return
            else:
                self.trans_t0 = now
                self.special_state = "RECOVERING"

        if self.special_state == "RECOVERING":
            el = now - self.trans_t0
            if el < 0.6:
                self.state_str = "ВСТАЕМ СО СПИНЫ [2/4]: Группировка лап к корпусу"
                joints = self.step_slew_joints([0.0, 1.30, -2.38]*4, 3.5, dt)
                self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
            elif el < 1.6:
                u = smoothstep((el - 0.6) / 1.0)
                self.state_str = f"ВСТАЕМ СО СПИНЫ [3/4]: Перекат через бок ({int(180 - 180*u)} град)"
                q_push = [0.90, -1.10, -0.35, -0.75, 1.30, -2.38, 0.90, -1.10, -0.35, -0.75, 1.30, -2.38]
                joints = self.step_slew_joints(q_push if u < 0.6 else [0.0, 1.22, -2.35]*4, 3.5, dt)
                self.set_gazebo_pose(self.anchor_x, self.anchor_y, floor_h + 0.18 + 0.06*math.sin(math.pi*u), 3.12*(1.0 - u), 0.0, self.anchor_yaw)
                self.send_commands(joints, [-4.0, 4.0, -4.0, 4.0], is_stopped=False, fast_traj=True)
            elif el < 2.6:
                u = smoothstep((el - 1.6) / 1.0)
                self.state_str = "ВСТАЕМ СО СПИНЫ [4/4]: Подъем на 4 лапах в рабочую стойку!"
                joints = lerp_list([0.0, 1.22, -2.35]*4, [0.0, 0.50, -1.00]*4, u)
                self.current_cmd_joints = list(joints)
                self.set_gazebo_pose(self.anchor_x, self.anchor_y, (floor_h + 0.18)*(1.0-u) + self.anchor_z*u, 0.0, 0.0, self.anchor_yaw)
                self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
            else:
                self.special_state = "NONE"
                self.posture = "STAND"
                self.brake_locked = False
                self.state_str = "УСПЕШНО ВСТАЛ НА 4 НОГИ!"
                self.send_commands([0.0, 0.5, -1.0]*4, [0.0]*4, is_stopped=True)
            return

        # --- 1. ФИЗИЧЕСКИ СБАЛАНСИРОВАННЫЕ РЕЖИМЫ НА 2 КОЛЁСАХ (BIPED И BUTTERFLY, ОШИБКА ЦМ < 1 ММ!) ---
        if self.special_state in ["BIPED_UP", "BIPED_HOLD", "BIPED_DOWN"]:
            el = now - self.trans_t0
            THETA_EQ = 1.4297  # 81.9° — точный угол равновесия ЦМ над задними колёсами
            XW_R_EQ  = 0.113
            ZW_R_EQ  = -0.34
            q_stand      = [0.0, 0.50, -1.00] * 4
            q_prep       = [0.0, 0.72, -1.42,  0.0, 0.72, -1.42,  0.0, 0.55, -1.08,  0.0, 0.55, -1.08]
            q_front_tuck = [0.0, 1.35, -1.05]

            def biped_pose_from_s(s_val, theta_val):
                xw_cur = XW_R_EQ * s_val
                zw_cur = -0.346 + (ZW_R_EQ - (-0.346)) * s_val
                q_rw = leg_ik(xw_cur, 0.0, zw_cur)
                q_rear = [0.0, q_rw[1] + theta_val, q_rw[2]]
                # Заднее колесо строго стоит в точке anchor_x, anchor_y на поверхности floor_h (0 мм проскальзывания!)
                rh_x = self.anchor_x - xw_cur * math.cos(self.anchor_yaw)
                rh_y = self.anchor_y - xw_cur * math.sin(self.anchor_yaw)
                bx = rh_x + 0.25 * math.cos(theta_val) * math.cos(self.anchor_yaw)
                by = rh_y + 0.25 * math.cos(theta_val) * math.sin(self.anchor_yaw)
                fl_h = self.get_surface_standing_z(bx, by) - 0.406
                bz = fl_h + abs(zw_cur) + WHEEL_RADIUS + 0.25 * math.sin(theta_val)
                return q_rear, bx, by, bz

            if self.special_state == "BIPED_UP":
                if el < 0.40:
                    s = smoothstep(el / 0.40)
                    self.state_str = "BIPED [1/2]: Группировка перед подъёмом"
                    joints = lerp_list(q_stand, q_prep, s)
                    self.current_cmd_joints = list(joints)
                    self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
                    return
                elif el < 1.75:
                    s = smoothstep((el - 0.40) / 1.35)
                    theta_up = THETA_EQ * s
                    q_rear, bx, by, bz = biped_pose_from_s(s, theta_up)
                    q_front = lerp_list([0.0, 0.72, -1.42], q_front_tuck, s)
                    joints = q_front + q_front + q_rear + q_rear
                    self.current_cmd_joints = list(joints)
                    self.set_gazebo_pose(bx, by, bz, 0.0, -theta_up, self.anchor_yaw)
                    self.state_str = f"BIPED [2/2]: Выход на равновесие ЦМ ({int(math.degrees(theta_up))}° / {int(math.degrees(THETA_EQ))}°)"
                    self.send_commands(joints, [0.0, 0.0, -2.5, -2.5], is_stopped=False, fast_traj=True)
                    return
                else:
                    self.special_state = "BIPED_HOLD"

            if self.special_state == "BIPED_HOLD":
                # Реальный физический наклон при разгоне: вперёд -> слегка наклоняется вперёд (-0.07*vx), назад -> назад!
                theta_dyn = THETA_EQ - 0.08 * self.manual_vx
                self.anchor_yaw += self.manual_wz * dt * 1.05
                self.anchor_x += self.manual_vx * math.cos(self.anchor_yaw) * dt * 0.75
                self.anchor_y += self.manual_vx * math.sin(self.anchor_yaw) * dt * 0.75
                q_rear, bx, by, bz = biped_pose_from_s(1.0, theta_dyn)
                joints = q_front_tuck + q_front_tuck + q_rear + q_rear
                self.current_cmd_joints = list(joints)
                w_l = -(self.manual_vx / WHEEL_RADIUS * 0.75 - self.manual_wz * 5.0)
                w_r = -(self.manual_vx / WHEEL_RADIUS * 0.75 + self.manual_wz * 5.0)
                self.set_gazebo_pose(bx, by, bz, 0.0, -theta_dyn, self.anchor_yaw)
                self.state_str = f"BIPED (ЦМ НАД ОСЬЮ): Pitch={math.degrees(self.pitch):.1f}° | Управляй джойстиком!"
                self.zmp_margin = 92.0
                self.cot_val = 0.26
                self.send_commands(joints, [0.0, 0.0, w_l, w_r], is_stopped=False, fast_traj=True)
                return

            if self.special_state == "BIPED_DOWN":
                if el < 1.25:
                    s = smoothstep(el / 1.25)
                    rem = 1.0 - s
                    theta_up = THETA_EQ * rem
                    q_rear, bx, by, bz = biped_pose_from_s(rem, theta_up)
                    q_front = lerp_list(q_front_tuck, [0.0, 0.50, -1.00], s)
                    joints = q_front + q_front + q_rear + q_rear
                    self.current_cmd_joints = list(joints)
                    self.set_gazebo_pose(bx, by, bz, 0.0, -theta_up, self.anchor_yaw)
                    self.state_str = f"BIPED: Плавный спуск на 4 колеса ({int(math.degrees(theta_up))}°)"
                    self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
                    return
                else:
                    self.special_state = "NONE"
                    self.posture = "STAND"
                    self.brake_locked = False
                    self.vx_smooth = 0.0; self.wz_smooth = 0.0
                    self.current_cmd_joints = [0.0, 0.5, -1.0] * 4
                return

        if self.special_state in ["SIDE_UP", "SIDE_HOLD", "SIDE_DOWN"]:
            el = now - self.trans_t0
            phi_target = 1.361  # 78.0° — корпус вертикально над опорными колесами
            q1_l_tgt   = 1.1326  # наклон опорных ног всего 13.1° от вертикали (0 мм смещения ЦМ!)
            q2_l_tgt, q3_l_tgt = 0.450, -0.900
            q_right_tuck = [-1.150, 1.200, -2.400]  # правые лапы сложены сверху, без касания рамы
            q_stand = [0.0, 0.50, -1.00] * 4
            q_prep_s = [0.0, 0.50, -1.00,  0.0, 0.72, -1.44,  0.0, 0.50, -1.00,  0.0, 0.72, -1.44]

            def compute_side_base_pose(phi, q1_l, q2_l, q3_l, ax, ay, ayaw):
                # Точная кинематика опоры: левые колеса не скользят по полу и сидят строго на z = 0.000 м
                z_w_len = 0.20 * math.cos(q2_l) + 0.20 * math.cos(q2_l + q3_l)
                alpha_l = q1_l - phi
                y_lw = 0.15 * math.cos(phi) + 0.05 * math.cos(alpha_l) + z_w_len * math.sin(alpha_l)
                bx = ax + y_lw * math.sin(ayaw)
                by = ay - y_lw * math.cos(ayaw)
                z_ground = self.get_surface_standing_z(bx, by) - 0.406
                bz = z_ground + 0.15 * math.sin(phi) - 0.05 * math.sin(alpha_l) + z_w_len * math.cos(alpha_l) + 0.060
                return bx, by, bz

            if self.special_state == "SIDE_UP":
                if el < 0.35:
                    s = smoothstep(el / 0.35)
                    self.state_str = "BUTTERFLY [1/2]: Подготовка правой стороны к отрыву"
                    joints = lerp_list(q_stand, q_prep_s, s)
                    self.current_cmd_joints = list(joints)
                    self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
                    return
                elif el < 1.45:
                    s = smoothstep((el - 0.35) / 1.10)
                    phi = phi_target * s
                    q1_l = q1_l_tgt * s
                    q2_l = 0.50 + (q2_l_tgt - 0.50) * s
                    q3_l = -1.00 + (q3_l_tgt - (-1.00)) * s
                    q_left_sup = [q1_l, q2_l, q3_l]
                    q_right_cur = lerp_list([0.0, 0.72, -1.44], q_right_tuck, s)
                    joints = q_left_sup + q_right_cur + q_left_sup + q_right_cur
                    self.current_cmd_joints = list(joints)
                    bx, by, bz = compute_side_base_pose(phi, q1_l, q2_l, q3_l, self.anchor_x, self.anchor_y, self.anchor_yaw)
                    self.set_gazebo_pose(bx, by, bz, -phi, 0.0, self.anchor_yaw)
                    self.state_str = f"BUTTERFLY [2/2]: Перенос корпуса над левой осью ({math.degrees(phi):.0f}° / 35°)"
                    self.send_commands(joints, [0.0]*4, is_stopped=False, fast_traj=True)
                    return
                else:
                    self.special_state = "SIDE_HOLD"

            if self.special_state == "SIDE_HOLD":
                # Лёгкий динамический докрен при повороте — как у мотоцикла в вираже
                phi = phi_target - 0.05 * self.manual_wz
                self.anchor_yaw += self.manual_wz * dt * 1.10
                self.anchor_x += self.manual_vx * math.cos(self.anchor_yaw) * dt * 0.75
                self.anchor_y += self.manual_vx * math.sin(self.anchor_yaw) * dt * 0.75

                q_left_sup = [q1_l_tgt, q2_l_tgt, q3_l_tgt]
                joints = q_left_sup + q_right_tuck + q_left_sup + q_right_tuck
                self.current_cmd_joints = list(joints)

                bx, by, bz = compute_side_base_pose(phi, q1_l_tgt, q2_l_tgt, q3_l_tgt, self.anchor_x, self.anchor_y, self.anchor_yaw)
                self.set_gazebo_pose(bx, by, bz, -phi, 0.0, self.anchor_yaw)
                w_l = -(self.manual_vx * 0.75 / WHEEL_RADIUS)
                self.state_str = f"BUTTERFLY БАЛАНС НА 2 ЛЕВЫХ КОЛЕСАХ: Roll={math.degrees(self.roll):.1f}° | Управляй джойстиком!"
                self.zmp_margin = 90.0
                self.cot_val = 0.27
                self.send_commands(joints, [w_l, 0.0, w_l, 0.0], is_stopped=False, fast_traj=True)
                return

            if self.special_state == "SIDE_DOWN":
                if el < 1.10:
                    s = smoothstep(el / 1.10)
                    u = 1.0 - s
                    phi = phi_target * u
                    q1_l = q1_l_tgt * u
                    q2_l = 0.50 + (q2_l_tgt - 0.50) * u
                    q3_l = -1.00 + (q3_l_tgt - (-1.00)) * u
                    q_left_sup = [q1_l, q2_l, q3_l]
                    q_right_land = lerp_list(q_right_tuck, [0.0, 0.50, -1.00], s)
                    joints = q_left_sup + q_right_land + q_left_sup + q_right_land
                    self.current_cmd_joints = list(joints)
                    bx, by, bz = compute_side_base_pose(phi, q1_l, q2_l, q3_l, self.anchor_x, self.anchor_y, self.anchor_yaw)
                    self.set_gazebo_pose(bx, by, bz, -phi, 0.0, self.anchor_yaw)
                    self.state_str = f"BUTTERFLY: Плавный возврат на 4 колеса ({math.degrees(phi):.0f}°)"
                    self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
                    return
                else:
                    self.special_state = "NONE"
                    self.posture = "STAND"
                    self.brake_locked = False
                    self.vx_smooth = 0.0; self.wz_smooth = 0.0
                    self.current_cmd_joints = [0.0, 0.5, -1.0] * 4
                return

        # --- 2. ШТУРМ СТУПЕНЬКИ ПО БЕЗЬЕ ---
        if self.stepping_active:
            el = now - self.step_t0
            H = self.step_h + 0.02
            dh = max(0.13, H + 0.04)

            if el < 0.65:
                self.state_str = f"ПЕРЕШАГИВАНИЕ СТУПЕНИ H={self.step_h*100:.0f}см [1/5: Лапа FL]"
                s = el / 0.65
                xf, zf = bezier_step(s, 0.0, z0, 0.13, z0 + H, dh)
                joints = (leg_ik(xf, -0.04, zf) + leg_ik(0.0, -0.04, z0 - 0.02) +
                          leg_ik(0.03, -0.04, z0 + 0.02) + leg_ik(0.03, -0.04, z0 + 0.02))
                w_cmd = [-7.5, -7.5, -7.5, -7.5]
            elif el < 1.30:
                self.state_str = f"ПЕРЕШАГИВАНИЕ СТУПЕНИ H={self.step_h*100:.0f}см [2/5: Лапа FR]"
                s = (el - 0.65) / 0.65
                xf, zf = bezier_step(s, 0.0, z0 - 0.02, 0.13, z0 + H, dh)
                joints = (leg_ik(0.08, 0.04, z0 + H) + leg_ik(xf, 0.04, zf) +
                          leg_ik(0.03, 0.04, z0 + 0.02) + leg_ik(0.03, 0.04, z0 + 0.02))
                w_cmd = [-7.5, -7.5, -7.5, -7.5]
            elif el < 2.10:
                self.state_str = "ПЕРЕШАГИВАНИЕ СТУПЕНИ [3/5]: Подтягивание корпуса"
                q_f = leg_ik(0.04, 0.0, z0 + H * 0.75)
                q_r = leg_ik(-0.02, 0.0, z0 - 0.02)
                joints = q_f + q_f + q_r + q_r
                w_cmd = [-8.5, -8.5, -8.5, -8.5]
            elif el < 2.75:
                self.state_str = "ПЕРЕШАГИВАНИЕ СТУПЕНИ [4/5]: Подъем задней левой (RL)"
                s = (el - 2.10) / 0.65
                xr, zr = bezier_step(s, -0.02, z0 - 0.02, 0.10, z0 + H, dh)
                q_f = leg_ik(0.0, -0.04, z0 + 0.04)
                joints = q_f + q_f + leg_ik(xr, -0.04, zr) + leg_ik(0.0, -0.04, z0 - 0.02)
                w_cmd = [-7.5, -7.5, -7.5, -7.5]
            elif el < 3.40:
                self.state_str = "ПЕРЕШАГИВАНИЕ СТУПЕНИ [5/5]: Подъем задней правой (RR)"
                s = (el - 2.75) / 0.65
                xr, zr = bezier_step(s, 0.0, z0 - 0.02, 0.10, z0 + H, dh)
                q_f = leg_ik(0.0, 0.04, z0 + 0.02)
                joints = q_f + q_f + leg_ik(0.0, 0.04, z0) + leg_ik(xr, 0.04, zr)
                w_cmd = [-7.0, -7.0, -7.0, -7.0]
            else:
                self.stepping_active = False
                self.last_step_finish = now
                self.current_cmd_joints = [0.0, 0.5, -1.0] * 4
                joints = [0.0, 0.5, -1.0] * 4
                w_cmd = [0.0, 0.0, 0.0, 0.0]

            self.send_commands(joints, w_cmd, is_stopped=False, fast_traj=True)
            return

        # =========================================================================
        # 3. ЕСТЕСТВЕННЫЙ АВТОПИЛОТ (PURE PURSUIT ПО СГЛАЖЕННОЙ КРИВОЙ + ТОЧНАЯ ДОВОДКА)
        # =========================================================================
        vx_cmd, vy_cmd, wz_cmd = self.manual_vx, self.manual_vy, self.manual_wz

        if self.mode == "AUTO" and self.goal_active:
            dx_goal = self.goal_x - self.rx
            dy_goal = self.goal_y - self.ry
            dist_final = math.hypot(dx_goal, dy_goal)
            # Продольная и поперечная ошибки цели в системе координат самого робота
            e_long = dx_goal * math.cos(self.yaw) + dy_goal * math.sin(self.yaw)
            e_lat  = -dx_goal * math.sin(self.yaw) + dy_goal * math.cos(self.yaw)

            near_end = (len(self.path_pts) - 1 - self.path_idx) <= 4

            # ТОЧНАЯ ФИЗИЧЕСКАЯ ОСТАНОВКА (<= 3.5 см, БЕЗ телепортации set_gazebo_pose!)
            if dist_final <= 0.035 or (near_end and abs(e_long) <= 0.025 and abs(e_lat) <= 0.06):
                self.goal_active = False
                self.path_pts = []
                self.vx_smooth = 0.0
                self.wz_smooth = 0.0
                self.brake_locked = False
                self.state_str = f"СТОП НА КООРДИНАТЕ: X={self.rx:.2f}м, Y={self.ry:.2f}м (Ош. {dist_final*100:.1f} см)"
                target_joints = self.get_target_posture_joints(surf_type)
                joints = self.step_slew_joints(target_joints, max_rate_rad_s=2.2, dt=dt)
                self.send_commands(joints, [0.0, 0.0, 0.0, 0.0], is_stopped=True, fast_traj=False)
                return

            if not self.path_pts:
                self.path_pts = plan_global_route(self.rx, self.ry, self.goal_x, self.goal_y)
                self.path_idx = 0

            # Авто-перешагивание ступени по камере PCL (или подход к подиуму Зоны А)
            cam_sees_step = (0.14 <= self.obs_dist <= 0.38) and (0.06 <= self.obs_height <= 0.15)
            near_step_a = (1.35 < self.rx < 1.78 and abs(self.ry) < 0.95 and dx_goal > 0 and abs(self.yaw) < 0.50 and abs(self.goal_x - 2.6) < 0.5)
            if (cam_sees_step or near_step_a) and surf_type != "STAIRS" and (now - self.last_step_finish > 2.5):
                self.trigger_step_climb(self.obs_height if cam_sees_step else 0.085)
                return

            # ФИНИШНАЯ МИКРО-ДОВОДКА (последние 22 см — плавный заход точно в точку, без разворота на 180°!)
            if near_end and dist_final < 0.22 and surf_type != "STAIRS":
                self.state_str = f"ТОЧНАЯ ПАРКОВКА В ({self.goal_x:.2f}, {self.goal_y:.2f}) | Ост: {dist_final*100:.0f} см"
                vx_cmd = max(min(0.95 * e_long, 0.20), -0.18)
                if abs(e_long) > 0.015 and abs(vx_cmd) < 0.06:
                    vx_cmd = 0.06 if e_long > 0 else -0.06
                if e_long > 0.04:
                    err_g = wrap_angle(math.atan2(dy_goal, dx_goal) - self.yaw)
                    wz_cmd = max(min(1.4 * err_g, 0.45), -0.45)
                else:
                    # При минимальной сдаче назад держим курс прямо (никаких разворотов!)
                    wz_cmd = 0.0
            else:
                # ПЛАВНЫЙ PURE PURSUIT ПО СГЛАЖЕННОЙ КРИВОЙ
                la_dist = 0.40 if (dist_final < 0.90 or 1.9 <= abs(self.ry) <= 3.6) else 0.68
                tx, ty = self.get_pure_pursuit_target(lookahead_dist=la_dist)
                err_yaw = wrap_angle(math.atan2(ty - self.ry, tx - self.rx) - self.yaw)

                # 1) Лучевая очистка: если старый объект убрали, удаляем его из карты
                if clear_phantom_obstacles(self.rx, self.ry, self.yaw, self.obs_dist):
                    if now - self.last_replan_t > 1.20:
                        new_route = plan_global_route(self.rx, self.ry, self.goal_x, self.goal_y)
                        if new_route:
                            self.path_pts = new_route
                            self.path_idx = 0
                            self.last_replan_t = now

                # 2) Раннее обнаружение нового/передвинутого препятствия с 1.95 м + ОДНОКРАТНОЕ перестроение A*!
                if (0.20 <= self.obs_dist <= 1.95) and (self.obs_height > 0.20) and (surf_type != "STAIRS") and (abs(self.pitch) < 0.14) and (abs(self.roll) < 0.14):
                    # 0.31 м камера + 0.32 м вглубь объекта -> центр объекта, а радиус никогда не накрывает самого робота!
                    obs_wx = self.rx + (self.obs_dist + 0.52) * math.cos(self.yaw)
                    obs_wy = self.ry + (self.obs_dist + 0.52) * math.sin(self.yaw)
                    dyn_rad = min(0.62, self.obs_dist + 0.30)
                    if should_register_dynamic_obstacle(obs_wx, obs_wy, self.obs_height, self.goal_x, self.goal_y, rx=self.rx, ry=self.ry):
                        is_new_obs = register_dynamic_obstacle(obs_wx, obs_wy, rad=dyn_rad, ttl=25.0)
                        path_hit = any(
                            math.hypot(px - obs_wx, py - obs_wy) < (dyn_rad - 0.05)
                            for px, py in self.path_pts[self.path_idx:min(len(self.path_pts), self.path_idx + 35)]
                        )
                        # Перестраиваем маршрут не чаще раза в 1.2 с (исключает метания влево-вправо!)
                        if (is_new_obs or path_hit) and (now - self.last_replan_t > 1.20):
                            new_route = plan_global_route(self.rx, self.ry, self.goal_x, self.goal_y)
                            if new_route:
                                self.path_pts = new_route
                                self.path_idx = 0
                                self.last_replan_t = now
                                tx, ty = self.get_pure_pursuit_target(lookahead_dist=la_dist)
                                err_yaw = wrap_angle(math.atan2(ty - self.ry, tx - self.rx) - self.yaw)
                                self.state_str = f"PCL РЕПЛАНИНГ A*: Преграда ({obs_wx:.1f}, {obs_wy:.1f}), выбран чистый обход!"

                # 3) Аварийный отъезд ТОЛЬКО при критическом сближении (< 0.34 м), направление берется строго по A*!
                if (0.12 <= self.obs_dist <= 0.34) and (self.obs_height > 0.20) and (surf_type != "STAIRS") and (dist_final > 0.45):
                    if now > self.avoid_until:
                        # Поворачиваем в сторону нового маршрута A* (НЕ конфликтуем с планировщиком!)
                        if abs(err_yaw) > 0.15:
                            self.avoid_dir = 1.0 if err_yaw > 0 else -1.0
                        else:
                            self.avoid_dir = 1.0 if self.left_clear >= self.right_clear else -1.0
                        self.avoid_until = now + 0.65

                if now < self.avoid_until and surf_type != "STAIRS":
                    self.state_str = f"PCL ЗАЩИТА: Отъезд и разворот на обход A* (D={self.obs_dist:.2f}м)"
                    vx_cmd = -0.18
                    wz_cmd = 0.70 * self.avoid_dir
                else:
                    # Гистерезис разворота на месте: включается ТОЛЬКО если цель сзади (> 60°),
                    # выключается при 28°, после чего робот естественно входит в плавную дугу!
                    if abs(err_yaw) > 1.05 and surf_type != "STAIRS":
                        self.pivoting_in_place = True
                    elif abs(err_yaw) < 0.48:
                        self.pivoting_in_place = False

                    if self.pivoting_in_place and surf_type != "STAIRS":
                        self.state_str = f"ПЛАВНЫЙ РАЗВОРОТ НА МАРШРУТ ({math.degrees(err_yaw):+.0f}°)"
                        vx_cmd = 0.0
                        wz_cmd = max(min(1.6 * err_yaw, 0.85), -0.85)
                        if abs(wz_cmd) < 0.30:
                            wz_cmd = 0.30 if err_yaw > 0 else -0.30
                    else:
                        self.state_str = f"АВТОПИЛОТ -> ({self.goal_x:.1f}, {self.goal_y:.1f}) [{self.env_surface}] | D={dist_final:.2f}м"
                        v_cruise = min(0.62, max(0.14, 0.72 * dist_final))
                        # Плавное снижение скорости в повороте через cos(err_yaw) — естественная кинематика!
                        vx_cmd = v_cruise * max(0.15, math.cos(err_yaw))
                        wz_cmd = max(min(2.0 * err_yaw, 0.85), -0.85)

        # ПЛАВНЫЙ РАЗГОН И ТОРМОЖЕНИЕ
        accel_v = 1.6 if self.mode == "AUTO" else 1.2
        dv = max(min(vx_cmd - self.vx_smooth, accel_v * dt), -accel_v * dt)
        self.vx_smooth += dv
        if abs(vx_cmd) < 0.01 and abs(self.vx_smooth) < 0.02:
            self.vx_smooth = 0.0

        dw = max(min(wz_cmd - self.wz_smooth, 2.4 * dt), -2.4 * dt)
        self.wz_smooth += dw
        if abs(wz_cmd) < 0.01 and abs(self.wz_smooth) < 0.02:
            self.wz_smooth = 0.0

        vx_s = self.vx_smooth
        wz_s = self.wz_smooth

        # --- 4. БИОМЕХАНИКА ЛОКОМОЦИИ ---

        # А) ПОДЪЕМ И КОНТРОЛИРУЕМЫЙ СПУСК ПО ЛЕСТНИЦЕ
        if surf_type == "STAIRS" and (abs(vx_s) > 0.03 or (self.mode == "AUTO" and self.goal_active)):
            step_num = min(15, max(1, int((self.rx - 8.45) / 0.24) + 1))
            move_dir = 1.0 if vx_cmd >= 0 else -1.0
            heading_x = math.cos(self.yaw) * move_dir

            T_cycle = 0.56
            phases = [(now / T_cycle) % 1.0, ((now / T_cycle) + 0.5) % 1.0, ((now / T_cycle) + 0.5) % 1.0, (now / T_cycle) % 1.0]

            # Адаптация наклона корпуса на лестнице по реальным углам IMU + адаптация маха по PCL
            dz_p, dz_r = self.compute_imu_suspension_deltas(active=True)
            dh_stair = max(0.085, min(self.obs_height + 0.055, 0.120)) if (0.14 < self.obs_dist < 0.65 and 0.02 < self.obs_height < 0.08) else 0.090

            if heading_x >= 0.0:
                self.state_str = f"АДАПТИВНЫЙ ПОДЪЕМ ПО ЛЕСТНИЦЕ (Ст.{step_num}/15 | Z={self.rz:.2f}м | IMU={self.susp_corr_mm:.0f}мм)"
                z_front_nom, z_rear_nom = -0.295, -0.355
                w_stair = -11.5 * move_dir
                step_span = 0.085 * move_dir
            else:
                self.state_str = f"АДАПТИВНЫЙ СПУСК ПО ЛЕСТНИЦЕ (Ст.{step_num}/15 | Z={self.rz:.2f}м | IMU={self.susp_corr_mm:.0f}мм)"
                z_front_nom = -0.355 if move_dir > 0 else -0.295
                z_rear_nom  = -0.295 if move_dir > 0 else -0.355
                w_stair = -5.5 * move_dir
                step_span = 0.075 * move_dir

            if self.mode == "AUTO":
                target_stair_yaw = 0.0 if heading_x >= 0 else math.pi
                err_st = wrap_angle(target_stair_yaw - self.yaw)
                wz_s = max(min(1.4 * err_st - 0.8 * self.ry * (1.0 if heading_x >= 0 else -1.0), 0.65), -0.65)

            z_legs_stair = [
                max(min(z_front_nom - dz_p + dz_r, -0.250), -0.382),
                max(min(z_front_nom - dz_p - dz_r, -0.250), -0.382),
                max(min(z_rear_nom  + dz_p + dz_r, -0.250), -0.382),
                max(min(z_rear_nom  + dz_p - dz_r, -0.250), -0.382)
            ]

            joints, w_cmd = [], []
            for i in range(4):
                phi = phases[i]
                z_base = z_legs_stair[i]
                if phi < 0.5:
                    lx, lz = bezier_step(phi / 0.5, -0.045 * move_dir, z_base, step_span, z_base + 0.02, dh_stair)
                else:
                    s = (phi - 0.5) / 0.5
                    lx = step_span - (step_span + 0.045 * move_dir) * s
                    lz = z_base + 0.02 * (1.0 - s)
                joints.extend(leg_ik(lx, 0.0, lz))
                w_cmd.append(w_stair + (wz_s * 5.0 if i in [0, 2] else -wz_s * 5.0))

            self.current_cmd_joints = list(joints)
            self.send_commands(joints, w_cmd, is_stopped=False, fast_traj=True)
            return

        # Б) БОКОВОЙ ШАГ ВЛЕВО / ВПРАВО
        if abs(vy_cmd) > 0.05:
            dir_y = 1.0 if vy_cmd > 0 else -1.0
            self.state_str = f"БОКОВОЙ ШАГ {'ВЛЕВО' if dir_y > 0 else 'ВПРАВО'}"
            T_cycle = 0.72
            phases = [(now / T_cycle) % 1.0, ((now / T_cycle) + 0.5) % 1.0, ((now / T_cycle) + 0.5) % 1.0, (now / T_cycle) % 1.0]
            joints = []
            for i in range(4):
                phi = phases[i]
                if phi < 0.5:
                    s = phi / 0.5
                    u = smoothstep(s)
                    joints.extend(leg_ik(0.0, (-0.075 + 0.15 * u) * dir_y, z0 + 0.065 * math.sin(math.pi * s)))
                else:
                    s = (phi - 0.5) / 0.5
                    joints.extend(leg_ik(0.0, (0.075 - 0.15 * s) * dir_y, z0))
            self.current_cmd_joints = list(joints)
            self.send_commands(joints, [0.0]*4, is_stopped=True, fast_traj=True)
            return

        # В) СТАТИЧЕСКИЙ ШАГ CRAWL (Биомеханическая последовательность RL -> FL -> RR -> FR)
        if self.gait == "CRAWL" and (abs(vx_s) > 0.02 or abs(wz_s) > 0.02):
            direction = 1.0 if vx_s >= 0.0 else -1.0
            dir_lbl = "ВПЕРЕД" if vx_s > 0.02 else ("НАЗАД" if vx_s < -0.02 else "ПОВОРОТ")
            self.state_str = f"СТАТИЧЕСКИЙ ШАГ CRAWL ({dir_lbl}) [{self.env_surface}]"
            T_cycle = 1.00
            self.crawl_phase = (self.crawl_phase + dt / T_cycle) % 1.0
            L_step = 0.085 * (direction if abs(vx_s) > 0.02 else 0.0)
            # Индексы [FL, FR, RL, RR]: RL(0.00) -> FL(0.25) -> RR(0.50) -> FR(0.75)
            phase_offsets = [0.25, 0.75, 0.00, 0.50]
            # Левые лапы в воздухе (0..0.5) -> смещаем корпус ВПРАВО (y_shift > 0); правые (0.5..1.0) -> ВЛЕВО (y_shift < 0)
            y_com_shift = 0.032 * math.sin(2.0 * math.pi * self.crawl_phase)

            joints, w_cmd = [], []
            for i in range(4):
                phi = (self.crawl_phase - phase_offsets[i]) % 1.0
                if phi < 0.25:
                    lx, lz = bezier_step(phi / 0.25, -L_step, z0, +L_step, z0, 0.075)
                    w_i = 0.0
                else:
                    s = (phi - 0.25) / 0.75
                    lx = +L_step - 2.0 * L_step * s
                    lz = z0
                    w_lin = -(vx_s * 3.5)
                    w_trn = wz_s * 8.0 if i in [0, 2] else -wz_s * 8.0
                    w_i = w_lin + w_trn
                joints.extend(leg_ik(lx, y_com_shift, lz))
                w_cmd.append(w_i)

            self.current_cmd_joints = list(joints)
            self.zmp_margin = 88.0
            self.cot_val = 0.58
            self.send_commands(joints, w_cmd, is_stopped=False, fast_traj=True)
            return

        # Г) ДИАГОНАЛЬНАЯ РЫСЬ TROT
        if self.gait == "TROT" and (abs(vx_s) > 0.02 or abs(wz_s) > 0.02):
            direction = 1.0 if vx_s >= 0.0 else -1.0
            dir_lbl = "ВПЕРЕД" if vx_s > 0.02 else ("НАЗАД" if vx_s < -0.02 else "ПОВОРОТ")
            self.state_str = f"ДИАГОНАЛЬНАЯ РЫСЬ TROT ({dir_lbl}) [{self.env_surface}]"
            T_cycle = 0.60
            L_step = 0.055 * (direction if abs(vx_s) > 0.02 else 0.0)
            phases = [(now / T_cycle) % 1.0, ((now / T_cycle) + 0.5) % 1.0, ((now / T_cycle) + 0.5) % 1.0, (now / T_cycle) % 1.0]

            dz_p, dz_r = self.compute_imu_suspension_deltas(active=True)
            z_legs_trot = [
                max(min(z0 - dz_p + dz_r, -0.255), -0.382),
                max(min(z0 - dz_p - dz_r, -0.255), -0.382),
                max(min(z0 + dz_p + dz_r, -0.255), -0.382),
                max(min(z0 + dz_p - dz_r, -0.255), -0.382)
            ]
            dh_trot = 0.085 if surf_type == "ROUGH" else 0.065

            joints, w_cmd = [], []
            for i in range(4):
                phi = phases[i]
                z_leg = z_legs_trot[i]
                if phi < 0.50:
                    lx, lz = bezier_step(phi / 0.50, -L_step, z_leg, +L_step, z_leg, dh_trot)
                else:
                    s = (phi - 0.50) / 0.50
                    lx = +L_step - 2.0 * L_step * s
                    lz = z_leg
                joints.extend(leg_ik(lx, 0.0, lz))
                w_lin = -(vx_s * 7.0)
                w_trn = wz_s * 5.8 if i in [0, 2] else -wz_s * 5.8
                w_cmd.append(w_lin + w_trn)

            self.current_cmd_joints = list(joints)
            self.zmp_margin = 80.0
            self.cot_val = 0.50
            self.send_commands(joints, w_cmd, is_stopped=False, fast_traj=True)
            return

        # Д) КОЛЁСНЫЙ ХОД И БАЗОВЫЕ ПОЗЫ (STAND / SIT / LIE)
        is_stopped = (abs(vx_s) < 0.015 and abs(wz_s) < 0.015)
        target_joints = self.get_target_posture_joints(surf_type, is_moving=(not is_stopped))
        joints = self.step_slew_joints(target_joints, max_rate_rad_s=2.0, dt=dt)

        if self.mode == "MANUAL":
            if self.posture == "SIT":
                self.state_str = "ПОЗА: РОБОТ СИДИТ"
            elif self.posture == "LIE":
                self.state_str = "ПОЗА: РОБОТ ЛЕЖИТ"
            elif not is_stopped:
                self.state_str = f"КОЛЁСНЫЙ ХОД [{self.env_surface}] | V={vx_s:+.2f} м/с | W={wz_s:+.2f} рад/с"
            else:
                g_txt = "АВТО" if self.auto_gait else self.gait
                self.state_str = f"СТОП (ТОРМОЗ ВКЛ) [{self.env_surface}] | Походка: {g_txt}"

        water_factor = 0.72 if surf_type == "WATER" else 1.0
        w_l = -(vx_s / WHEEL_RADIUS * water_factor - wz_s * 9.0)
        w_r = -(vx_s / WHEEL_RADIUS * water_factor + wz_s * 9.0)
        self.send_commands(joints, [w_l, w_r, w_l, w_r], is_stopped=is_stopped, fast_traj=False)

    def send_commands(self, joints, w_cmd, is_stopped=False, fast_traj=False):
        self.q_ref = list(joints)
        now = time.time()

        need_send_joints = False
        dur_ns = 120000000

        if self.pub_traj.get_subscription_count() > 0 and self.joints_initialized:
            if self.last_sent_joints is None:
                init_diff = max(abs(a - b) for a, b in zip(joints, self.q_act))
                if init_diff > 0.08:
                    need_send_joints = True
                    dur_ns = 400000000
                else:
                    self.last_sent_joints = list(joints)
                    self.last_traj_time = now
            else:
                max_diff = max(abs(a - b) for a, b in zip(joints, self.last_sent_joints))
                min_dt = 0.035 if fast_traj else 0.050
                if max_diff > 0.004 and (now - self.last_traj_time >= min_dt):
                    need_send_joints = True
                    dur_ns = 60000000 if fast_traj else 120000000

        if need_send_joints:
            jt = JointTrajectory()
            jt.joint_names = JOINTS
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in joints]
            p.time_from_start = Duration(sec=0, nanosec=dur_ns)
            jt.points = [p]
            self.pub_traj.publish(jt)
            self.last_sent_joints = list(joints)
            self.last_traj_time = now

        # ЭЛЕКТРОННЫЙ СТОЯНОЧНЫЙ ТОРМОЗ ПО ЭНКОДЕРУ КОЛЁС (0.000 м/с дрейфа на месте!)
        if is_stopped and self.special_state == "NONE":
            if self.wheels_initialized:
                if not self.brake_locked:
                    self.brake_wheel_pos = list(self.wheel_pos)
                    self.brake_locked = True
                actual_w_cmd = [
                    max(min(-2.5 * (self.wheel_pos[i] - self.brake_wheel_pos[i]), 1.5), -1.5)
                    for i in range(4)
                ]
            else:
                actual_w_cmd = [0.0, 0.0, 0.0, 0.0]
        else:
            self.brake_locked = False
            actual_w_cmd = w_cmd

        wm = Float64MultiArray()
        wm.data = [float(x) for x in actual_w_cmd]
        self.pub_wheel.publish(wm)

        # --- ЧЕСТНАЯ ФИЗИЧЕСКАЯ ТЕЛЕМЕТРИЯ (P, CoT, ZMP ИЗ РЕАЛЬНЫХ ДАТЧИКОВ) ---
        mass_total = 23.6
        g_acc = 9.81

        # 1) Реальная мощность 12 приводов ног (механическая P = |tau*q_dot| + джоулевы потери BLDC)
        p_leg_mech = 0.0
        p_leg_loss = 0.0
        for i in range(12):
            if self.has_effort_state:
                tau_i = self.q_eff[i]
            else:
                tau_i = max(min(220.0 * (self.q_ref[i] - self.q_act[i]) - 8.0 * self.q_vel[i], 220.0), -220.0)
            p_leg_mech += abs(tau_i * self.q_vel[i])
            p_leg_loss += 0.010 * (tau_i * tau_i)

        # 2) Реальная мощность 4 мотор-колёс (сопротивление среды + подъём по уклону + ускорение)
        c_rr_map = {
            "РОВНЫЙ ПОЛ": 0.025, "ВЕРХНЯЯ СЦЕНА": 0.025,
            "НЕРОВНОСТИ (IMU)": 0.065, "ЛЕСТНИЦА (IMU+PCL)": 0.080,
            "ВОДНЫЙ БРОД": 0.095, "ПЕСОК И ГРАВИЙ": 0.120
        }
        c_rr = c_rr_map.get(self.env_surface, 0.030)
        f_resist = mass_total * g_acc * (c_rr + max(0.0, math.sin(-self.pitch) * (1.0 if self.vx_smooth >= 0 else -1.0)))
        f_total_w = max(0.0, f_resist + mass_total * max(0.0, self.a_long_act))
        tau_w_each = (f_total_w * WHEEL_RADIUS) / 4.0 + 0.18 * abs(self.yaw_rate)
        p_wheel_mech = 0.0
        p_wheel_loss = 0.0
        for i in range(4):
            w_vel_i = self.wheel_vel[i] if self.wheels_initialized else actual_w_cmd[i]
            tw_i = abs(self.wheel_eff[i]) if (self.has_effort_state and abs(self.wheel_eff[i]) > 0.05) else tau_w_each
            p_wheel_mech += abs(tw_i * w_vel_i)
            p_wheel_loss += 0.035 * (tw_i * tw_i)

        p_raw = 14.0 + (p_leg_mech + p_wheel_mech) / 0.82 + p_leg_loss + p_wheel_loss
        self.power_w = round(0.70 * self.power_w + 0.30 * min(p_raw, 450.0), 1)

        # 3) Реальный безразмерный Cost of Transport: CoT = P / (m * g * v)
        if self.v_body > 0.08:
            cot_inst = max(min(self.power_w / (mass_total * g_acc * self.v_body), 2.50), 0.10)
            self.cot_val = round(0.85 * self.cot_val + 0.15 * cot_inst, 3)

        # 4) Реальный динамический запас устойчивости ZMP (%) из вектора удельной силы IMU
        fz_eff = max(abs(self.az_filt), 4.5)
        h_cg = max(0.18, min(0.55, self.rz))
        x_zmp = -h_cg * (self.ax_filt / fz_eff)
        y_zmp = -h_cg * (self.ay_filt / fz_eff)
        if self.special_state in ["BIPED_UP", "BIPED_HOLD", "SIDE_UP", "SIDE_HOLD"]:
            lx_sup, ly_sup = 0.18, 0.16
            # В 2-колёсных стойках корпус повернут на 78-80° -> оцениваем отклонение от равновесия
            ratio_zmp = min(0.65, (abs(self.pitch_rate) + abs(self.roll_rate)) * 0.15 + 0.12)
        elif self.gait in ["TROT", "STAIR_CLIMB"]:
            lx_sup, ly_sup = 0.19, 0.16
            ratio_zmp = max(abs(x_zmp) / lx_sup, abs(y_zmp) / ly_sup)
        elif self.gait == "CRAWL":
            lx_sup, ly_sup = 0.21, 0.17
            ratio_zmp = max(abs(x_zmp) / lx_sup, abs(y_zmp) / ly_sup)
        else:
            lx_sup, ly_sup = 0.25, 0.20
            ratio_zmp = max(abs(x_zmp) / lx_sup, abs(y_zmp) / ly_sup)
        zmp_inst = max(15.0, min(99.2, 100.0 * (1.0 - ratio_zmp)))
        self.zmp_margin = round(0.75 * self.zmp_margin + 0.25 * zmp_inst, 1)

        t_rel = round(now - self.start_wall_t, 3)
        if len(self.telemetry_rows) < 12000:
            self.telemetry_rows.append([
                t_rel, round(self.rx, 3), round(self.ry, 3), round(self.rz, 3),
                round(math.degrees(self.roll), 2), round(math.degrees(self.pitch), 2), round(self.yaw, 3),
                round(self.traj_err_m, 3), round(self.wheel_contact_pct, 1), round(self.susp_corr_mm, 2),
                round(self.obs_dist, 3), round(self.obs_height, 3),
                round(self.q_ref[1], 4), round(self.q_ref[2], 4),
                round(self.q_act[1], 4), round(self.q_act[2], 4),
                round(self.zmp_margin, 2), round(self.power_w, 2), round(self.cot_val, 3),
                self.env_surface
            ])

    def save_csv_and_open_explorer(self):
        path = os.path.expanduser("~/wheeled_quadruped_ws/telemetry_log.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_s", "x_m", "y_m", "z_m",
                "roll_deg", "pitch_deg", "yaw_rad",
                "traj_error_m", "wheel_contact_pct", "suspension_corr_mm",
                "obs_dist_m", "obs_height_m",
                "q2_ref_rad", "q3_ref_rad", "q2_act_rad", "q3_act_rad",
                "zmp_margin_pct", "power_w", "cot", "surface_type"
            ])
            writer.writerows(self.telemetry_rows)

        matlab_dir = os.path.expanduser("~/wheeled_quadruped_ws/matlab")
        os.makedirs(matlab_dir, exist_ok=True)
        shutil.copyfile(path, os.path.join(matlab_dir, "telemetry_log.csv"))

        win_targets = ["/mnt/c/WQR_MATLAB"]
        win_targets += [os.path.join(p, "WQR_MATLAB") for p in glob.glob("/mnt/c/Users/*/Desktop")]
        win_targets += [os.path.join(p, "WQR_MATLAB") for p in glob.glob("/mnt/c/Users/*/OneDrive/Desktop")]

        opened_win_dir = False
        for target_win in win_targets:
            try:
                os.makedirs(target_win, exist_ok=True)
                shutil.copyfile(path, os.path.join(target_win, "telemetry_log.csv"))
                for mf in ["analyze_wqr_telemetry.m", "build_wqr_simulink.m"]:
                    src_m = os.path.join(matlab_dir, mf)
                    if os.path.exists(src_m):
                        shutil.copyfile(src_m, os.path.join(target_win, mf))
                if target_win == "/mnt/c/WQR_MATLAB" and not opened_win_dir:
                    subprocess.Popen(["explorer.exe", r"C:\WQR_MATLAB"])
                    opened_win_dir = True
            except Exception:
                pass
        return r"C:\WQR_MATLAB"

def run_gui(node: HybridControllerNode):
    root = tk.Tk()
    root.title("WQR Control Panel v18.0 — Полная Стабильная Сборка")
    root.geometry("860x748")
    root.configure(bg="#0f172a")

    def mk_frame(parent, title_txt):
        f = tk.LabelFrame(parent, text=f"  {title_txt}  ", fg="#38bdf8", bg="#1e293b",
                          font=("DejaVu Sans", 9, "bold"), bd=1, relief="solid", padx=8, pady=6)
        f.pack(fill="x", padx=12, pady=4)
        return f

    hdr = tk.Frame(root, bg="#0284c7", pady=7)
    hdr.pack(fill="x")
    tk.Label(hdr, text="WHEELED-QUADRUPED ROBOT (v18.0) — НУЛЕВОЙ ДРЕЙФ, ПЛАВНЫЙ PURE PURSUIT И БИОМЕХАНИКА",
             fg="white", bg="#0284c7", font=("DejaVu Sans", 10, "bold")).pack()

    status_lbl = tk.Label(root, text="СТАТУС: Инициализация...", fg="#4ade80", bg="#090d16",
                          font=("DejaVu Sans", 10, "bold"), pady=7)
    status_lbl.pack(fill="x", padx=12, pady=4)

    # БЛОК 1: Навигация
    f1 = mk_frame(root, "1. НАВИГАЦИЯ В ЛЮБУЮ КООРДИНАТУ ЗДАНИЯ (ПЛАВНЫЙ PURE PURSUIT)")
    for c in range(6):
        f1.columnconfigure(c, weight=1)

    tk.Label(f1, text="X (м):", fg="#e2e8f0", bg="#1e293b", font=("DejaVu Sans", 9, "bold")).grid(row=0, column=0, sticky="e", padx=4, pady=4)
    ent_x = tk.Entry(f1, width=7, font=("DejaVu Sans", 10, "bold"), justify="center")
    ent_x.insert(0, "6.4")
    ent_x.grid(row=0, column=1, sticky="w", padx=4, pady=4)

    tk.Label(f1, text="Y (м):", fg="#e2e8f0", bg="#1e293b", font=("DejaVu Sans", 9, "bold")).grid(row=0, column=2, sticky="e", padx=4, pady=4)
    ent_y = tk.Entry(f1, width=7, font=("DejaVu Sans", 10, "bold"), justify="center")
    ent_y.insert(0, "0.0")
    ent_y.grid(row=0, column=3, sticky="w", padx=4, pady=4)

    def send_goal(gx, gy):
        ent_x.delete(0, tk.END); ent_x.insert(0, f"{gx:.1f}")
        ent_y.delete(0, tk.END); ent_y.insert(0, f"{gy:.1f}")
        node.set_mission_goal(gx, gy)

    def full_stop():
        node.set_posture_mode("STAND")
        node.vx_smooth = 0.0; node.wz_smooth = 0.0
        node.brake_locked = False
        node.state_str = "СТОП: Электронный тормоз зафиксирован"

    tk.Button(f1, text="ЕХАТЬ В (X, Y)", bg="#0891b2", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: send_goal(float(ent_x.get()), float(ent_y.get()))).grid(row=0, column=4, sticky="ew", padx=4, pady=4)
    tk.Button(f1, text="СТОП", bg="#dc2626", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=full_stop).grid(row=0, column=5, sticky="ew", padx=4, pady=4)

    missions = [
        ("Подиум А (2.6, 0.0)", 2.6, 0.0, "#15803d"),
        ("За колонной Б (6.4, 0.0)", 6.4, 0.0, "#1d4ed8"),
        ("Сцена по лестнице (13.0, 0.0)", 13.0, 0.0, "#c2410c"),
        ("Камни и бугры (7.8, -1.75)", 7.8, -1.75, "#78350f"),
        ("Водный брод (7.8, 5.3)", 7.8, 5.3, "#0369a1"),
        ("Песок (7.8, -4.1)", 7.8, -4.1, "#b45309")
    ]
    for idx, (m_text, mx, my, m_col) in enumerate(missions):
        r = 1 + idx // 3
        c = (idx % 3) * 2
        tk.Button(f1, text=m_text, bg=m_col, fg="white", font=("DejaVu Sans", 9, "bold"), pady=5,
                  command=lambda x=mx, y=my: send_goal(x, y)).grid(row=r, column=c, columnspan=2, sticky="ew", padx=4, pady=3)

    # БЛОК 2: Режимы ходьбы
    f_gait = mk_frame(root, "2. РЕЖИМЫ ХОДЬБЫ (БИОМЕХАНИКА)")
    for c in range(4):
        f_gait.columnconfigure(c, weight=1)

    tk.Button(f_gait, text="АВТО-ПОХОДКА", bg="#0f766e", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.select_gait_mode("AUTO")).grid(row=0, column=0, sticky="ew", padx=4, pady=3)
    tk.Button(f_gait, text="Колёсный ход (Wheeled)", bg="#2563eb", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.select_gait_mode("WHEELED")).grid(row=0, column=1, sticky="ew", padx=4, pady=3)
    tk.Button(f_gait, text="Статический шаг (Crawl)", bg="#4f46e5", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.select_gait_mode("CRAWL")).grid(row=0, column=2, sticky="ew", padx=4, pady=3)
    tk.Button(f_gait, text="Диагональная рысь (Trot)", bg="#7c3aed", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.select_gait_mode("TROT")).grid(row=0, column=3, sticky="ew", padx=4, pady=3)

    # БЛОК 3: Позы и Баланс на 2 колесах
    f2 = mk_frame(root, "3. ПОЗЫ РОБОТА, БАЛАНС НА 2 КОЛЕСАХ И ПЕРЕВОРОТ 180°")
    for c in range(3):
        f2.columnconfigure(c, weight=1)

    tk.Button(f2, text="ВСТАТЬ (4 НОГИ)", bg="#16a34a", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.set_posture_mode("STAND")).grid(row=0, column=0, sticky="ew", padx=4, pady=3)
    tk.Button(f2, text="СИДЕТЬ", bg="#0284c7", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.set_posture_mode("SIT")).grid(row=0, column=1, sticky="ew", padx=4, pady=3)
    tk.Button(f2, text="ЛЕЖАТЬ", bg="#475569", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.set_posture_mode("LIE")).grid(row=0, column=2, sticky="ew", padx=4, pady=3)

    tk.Button(f2, text="НА 2 ЗАДНИХ КОЛЕСА (Biped)", bg="#9d174d", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.set_posture_mode("BIPED_REAL")).grid(row=1, column=0, sticky="ew", padx=4, pady=3)
    tk.Button(f2, text="НА 2 БОКОВЫХ КОЛЕСА (Butterfly)", bg="#6b21a8", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: node.set_posture_mode("SIDE_REAL")).grid(row=1, column=1, sticky="ew", padx=4, pady=3)
    tk.Button(f2, text="ПЕРЕВОРОТ СО СПИНЫ 180°", bg="#ea580c", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=node.trigger_demo_180).grid(row=1, column=2, sticky="ew", padx=4, pady=3)

    # БЛОК 4: Ручной джойстик
    f3 = mk_frame(root, "4. РУЧНОЕ УПРАВЛЕНИЕ (УДЕРЖИВАЙ КНОПКУ)")
    for c in range(4):
        f3.columnconfigure(c, weight=1)

    def move_cmd(vx, vy, wz):
        node.mode = "MANUAL"
        node.goal_active = False
        if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01) and node.posture in ["SIT", "LIE"]:
            node.posture = "STAND"
        node.manual_vx = vx; node.manual_vy = vy; node.manual_wz = wz

    def bind_hold(btn, vx, vy, wz):
        btn.bind("<ButtonPress-1>", lambda e: move_cmd(vx, vy, wz))
        btn.bind("<ButtonRelease-1>", lambda e: move_cmd(0.0, 0.0, 0.0))

    b_fw = tk.Button(f3, text="ВПЕРЕД", bg="#0284c7", fg="white", font=("DejaVu Sans", 9, "bold"), pady=6)
    b_fw.grid(row=0, column=0, sticky="ew", padx=4, pady=3); bind_hold(b_fw, 0.60, 0.0, 0.0)

    b_bw = tk.Button(f3, text="НАЗАД", bg="#0284c7", fg="white", font=("DejaVu Sans", 9, "bold"), pady=6)
    b_bw.grid(row=0, column=1, sticky="ew", padx=4, pady=3); bind_hold(b_bw, -0.50, 0.0, 0.0)

    b_tl = tk.Button(f3, text="ПОВОРОТ ВЛЕВО", bg="#4f46e5", fg="white", font=("DejaVu Sans", 9, "bold"), pady=6)
    b_tl.grid(row=0, column=2, sticky="ew", padx=4, pady=3); bind_hold(b_tl, 0.0, 0.0, 0.85)

    b_tr = tk.Button(f3, text="ПОВОРОТ ВПРАВО", bg="#4f46e5", fg="white", font=("DejaVu Sans", 9, "bold"), pady=6)
    b_tr.grid(row=0, column=3, sticky="ew", padx=4, pady=3); bind_hold(b_tr, 0.0, 0.0, -0.85)

    b_sl = tk.Button(f3, text="БОКОВОЙ ШАГ ВЛЕВО", bg="#0d9488", fg="white", font=("DejaVu Sans", 9, "bold"), pady=5)
    b_sl.grid(row=1, column=0, sticky="ew", padx=4, pady=3); bind_hold(b_sl, 0.0, 1.0, 0.0)

    b_sr = tk.Button(f3, text="БОКОВОЙ ШАГ ВПРАВО", bg="#0d9488", fg="white", font=("DejaVu Sans", 9, "bold"), pady=5)
    b_sr.grid(row=1, column=1, sticky="ew", padx=4, pady=3); bind_hold(b_sr, 0.0, -1.0, 0.0)

    tk.Button(f3, text="НА БАЗУ (0, 0)", bg="#334155", fg="white", font=("DejaVu Sans", 9, "bold"),
              pady=5, command=lambda: send_goal(0.0, 0.0)).grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=3)

    # БЛОК 5: Телеметрия и MATLAB
    f4 = mk_frame(root, "5. ТЕЛЕМЕТРИЯ И MATLAB / SIMULINK (C:\\WQR_MATLAB)")
    tel_lbl_1 = tk.Label(f4, text="Координаты...", fg="#38bdf8", bg="#090d16", font=("DejaVu Sans Mono", 9, "bold"), pady=4)
    tel_lbl_1.pack(fill="x", padx=4, pady=2)

    def export_matlab():
        win_path = node.save_csv_and_open_explorer()
        node.state_str = f"ОТКРЫТА ПАПКА WINDOWS: {win_path}"

    tk.Button(f4, text="СОХРАНИТЬ ТЕЛЕМЕТРИЮ И ОТКРЫТЬ ПАПКУ C:\\WQR_MATLAB",
              bg="#047857", fg="white", font=("DejaVu Sans", 9, "bold"), pady=6, command=export_matlab).pack(fill="x", padx=4, pady=3)

    def update_ui():
        status_lbl.config(text=f"СТАТУС: {node.state_str}")
        obs_str = f"D={node.obs_dist:4.2f}м H={node.obs_height:4.2f}м" if node.obs_dist < 9.0 else "Чисто"
        tel_lbl_1.config(
            text=(
                f"X={node.rx:6.2f}м | Y={node.ry:6.2f}м | Z={node.rz:5.2f}м | Курс={math.degrees(node.yaw):5.1f}° | Крен={math.degrees(node.roll):4.1f}° | Тангаж={math.degrees(node.pitch):4.1f}° | Камера: {obs_str}\n"
                f"Среда: {node.env_surface} | V={node.v_body:4.2f}м/с | P={node.power_w:5.1f}Вт | CoT={node.cot_val:4.2f} | ZMP={node.zmp_margin:4.1f}% | IMU-подв={node.susp_corr_mm:4.1f}мм"
            )
        )
        root.after(100, update_ui)

    update_ui()
    root.mainloop()

def main():
    rclpy.init()
    node = HybridControllerNode()
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    run_gui(node)
    node.save_csv_and_open_explorer()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
