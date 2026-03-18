import os
import time
import math
from datetime import datetime

import cv2
import numpy as np
import cosysairsim as airsim

LEADER = "LeaderDrone"
FOLLOWER = "FollowerDrone"


def quat_to_yaw(q):
    # 四元数 -> yaw（绕Z轴）
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    w = q.w_val
    x = q.x_val
    y = q.y_val
    z = q.z_val
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def rot2d(vx, vy, yaw_rad):
    # 把局部二维向量 (vx,vy) 绕 Z 轴旋转 yaw，得到世界系二维向量
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    wx = c * vx - s * vy
    wy = s * vx + c * vy
    return wx, wy


def yaw_to_leader_deg(fx, fy, lx, ly):
    # atan2(dy, dx) -> degrees
    return math.degrees(math.atan2(ly - fy, lx - fx))


def is_finite(*vals):
    for v in vals:
        if not math.isfinite(v):
            return False
    return True


def is_finite_position(pos):
    return is_finite(pos.x_val, pos.y_val, pos.z_val)


def is_finite_quat(q):
    return is_finite(q.w_val, q.x_val, q.y_val, q.z_val)


def follower_control_step(client, cruise_z, offset_local_x, offset_local_y, dz, k_follow, vxy_max, vz_max):
    try:
        leader_state = client.getMultirotorState(vehicle_name=LEADER)
        follower_state = client.getMultirotorState(vehicle_name=FOLLOWER)
    except Exception as e:
        print(f"[告警] 读取状态失败，跳过本周期: {e}")
        return None

    lp = leader_state.kinematics_estimated.position
    lo = leader_state.kinematics_estimated.orientation
    fp = follower_state.kinematics_estimated.position

    if (not is_finite_position(lp)) or (not is_finite_quat(lo)) or (not is_finite_position(fp)):
        print("[告警] 状态含 NaN/Inf，跳过本周期")
        return None

    leader_yaw = quat_to_yaw(lo)
    if not is_finite(leader_yaw):
        print("[告警] leader_yaw 非有限值，跳过本周期")
        return None

    # 目标点：Follower 相对 Leader 固定偏移
    ox, oy = rot2d(offset_local_x, offset_local_y, leader_yaw)
    tx = lp.x_val + ox
    ty = lp.y_val + oy
    tz = lp.z_val + dz

    if not is_finite(tx, ty, tz):
        print("[告警] 目标点含 NaN/Inf，跳过本周期")
        return None

    ex = tx - fp.x_val
    ey = ty - fp.y_val
    ez = tz - fp.z_val

    if not is_finite(ex, ey, ez):
        print("[告警] 误差含 NaN/Inf，跳过本周期")
        return None

    vx_f = clamp(k_follow * ex, -vxy_max, vxy_max)
    vy_f = clamp(k_follow * ey, -vxy_max, vxy_max)
    vz_f = clamp(k_follow * ez, -vz_max, vz_max)

    yaw_deg = yaw_to_leader_deg(fp.x_val, fp.y_val, lp.x_val, lp.y_val)
    if not is_finite(vx_f, vy_f, vz_f, yaw_deg):
        print("[告警] 控制量含 NaN/Inf，跳过本周期")
        return None

    client.moveByVelocityAsync(
        vx_f,
        vy_f,
        vz_f,
        duration=0.1,
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw_deg),
        vehicle_name=FOLLOWER,
    )

    return lp


def capture_follower_rgb(client, waypoint_index, save_dir):
    raw = client.simGetImage("0", airsim.ImageType.Scene, vehicle_name=FOLLOWER)
    if raw is None:
        print(f"[告警] 航点 {waypoint_index} 拍照失败：返回 None")
        return None

    if isinstance(raw, str):
        raw_bytes = raw.encode("latin1")
    else:
        raw_bytes = raw

    if len(raw_bytes) == 0:
        print(f"[告警] 航点 {waypoint_index} 拍照失败：空字节")
        return None

    img_np = np.frombuffer(raw_bytes, dtype=np.uint8)
    img = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
    if img is None:
        print(f"[告警] 航点 {waypoint_index} 拍照失败：imdecode 结果为空")
        return None

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(save_dir, f"wp{waypoint_index}_{ts}.png")
    ok = cv2.imwrite(save_path, img)
    if not ok:
        print(f"[告警] 航点 {waypoint_index} 保存失败：{save_path}")
        return None

    return save_path


def sleep_to_rate(start_t, dt):
    # 保持 10Hz
    remain = dt - (time.time() - start_t)
    if remain > 0:
        time.sleep(remain)


if __name__ == "__main__":
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("连接完成----成功")

    # 控制参数
    cruise_z = -40.0
    dt = 0.1
    arrive_thresh = 1.5

    # Leader 矩形参数
    rect_dx = 40.0
    rect_dy = 25.0
    leader_speed = 5.0

    # Follower 跟随参数
    R = 8.0
    theta_deg = 45.0
    dz = 0.0
    k_follow = 0.8
    vxy_max = 8.0
    vz_max = 3.0

    # 约定 local x=前方，local y=右方（NED常见）
    # 左后方45°: x负、y负；如果你发现跑到右后方，把 left_back 改为 False
    left_back = True
    theta = math.radians(theta_deg)
    offset_local_x = -R * math.cos(theta)
    offset_local_y = (-1.0 if left_back else 1.0) * R * math.sin(theta)

    # 拍照目录
    save_dir = os.path.join(".", "captures")
    os.makedirs(save_dir, exist_ok=True)

    # API控制 + 解锁
    client.enableApiControl(True, vehicle_name=LEADER)
    client.armDisarm(True, vehicle_name=LEADER)
    client.enableApiControl(True, vehicle_name=FOLLOWER)
    client.armDisarm(True, vehicle_name=FOLLOWER)

    # 起飞 + 到巡检高度
    client.takeoffAsync(vehicle_name=LEADER).join()
    client.takeoffAsync(vehicle_name=FOLLOWER).join()
    client.moveToZAsync(cruise_z, 3, vehicle_name=LEADER).join()
    client.moveToZAsync(cruise_z, 3, vehicle_name=FOLLOWER).join()
    print("两机起飞完成----成功")

    # 读取 Leader 当前点作为 P0
    leader_state0 = client.getMultirotorState(vehicle_name=LEADER)
    p0 = leader_state0.kinematics_estimated.position
    if not is_finite_position(p0):
        raise RuntimeError("Leader 初始位置含 NaN/Inf，任务终止")

    x0 = p0.x_val
    y0 = p0.y_val
    z0 = cruise_z

    # 航点顺序：P0 -> P1 -> P2 -> P3 -> P0
    waypoints = [
        (x0, y0, z0),
        (x0 + rect_dx, y0, z0),
        (x0 + rect_dx, y0 + rect_dy, z0),
        (x0, y0 + rect_dy, z0),
        (x0, y0, z0),
    ]

    try:
        for i, (wx, wy, wz) in enumerate(waypoints):
            print(f"Leader 前往航点 {i}")
            client.moveToPositionAsync(wx, wy, wz, leader_speed, vehicle_name=LEADER)

            # 到点判定：距离阈值，避免 join 卡住
            while True:
                tick_start = time.time()
                lp = follower_control_step(
                    client,
                    cruise_z,
                    offset_local_x,
                    offset_local_y,
                    dz,
                    k_follow,
                    vxy_max,
                    vz_max,
                )

                if lp is not None:
                    dx = wx - lp.x_val
                    dy = wy - lp.y_val
                    dz_wp = wz - lp.z_val
                    if is_finite(dx, dy, dz_wp):
                        dist = math.sqrt(dx * dx + dy * dy + dz_wp * dz_wp)
                        if dist < arrive_thresh:
                            break
                    else:
                        print("[告警] 航点距离计算含 NaN/Inf，跳过本周期")

                sleep_to_rate(tick_start, dt)

            # 到达后悬停2秒，并让 Follower 立即拍一张
            client.hoverAsync(vehicle_name=LEADER)
            saved = capture_follower_rgb(client, i, save_dir)
            if saved is None:
                print(f"航点 {i} 到达，Follower 拍照失败")
            else:
                print(f"航点 {i} 到达，Follower 拍照保存：{saved}")

            hold_until = time.time() + 2.0
            while time.time() < hold_until:
                tick_start = time.time()
                follower_control_step(
                    client,
                    cruise_z,
                    offset_local_x,
                    offset_local_y,
                    dz,
                    k_follow,
                    vxy_max,
                    vz_max,
                )
                sleep_to_rate(tick_start, dt)

        print("矩形巡检任务完成----成功")
        client.hoverAsync(vehicle_name=LEADER)
        client.hoverAsync(vehicle_name=FOLLOWER)

    except KeyboardInterrupt:
        print("任务被手动中断，执行悬停----提示")
        client.hoverAsync(vehicle_name=LEADER)
        client.hoverAsync(vehicle_name=FOLLOWER)
