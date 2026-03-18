import cosysairsim as airsim
import numpy as np
import cv2
import time
import math

# -----------------------
# Config
# -----------------------
VEHICLE = "Drone1"
CAMERA = "0"

CRUISE_Z = -6.0          # 巡航高度（NED：负数=在空中）
DT = 0.1                 # 控制周期 10Hz

# 速度档位（m/s）
V_FAST = 2.5
V_MED  = 1.2
V_SLOW = 0.4

# 安全距离阈值（米）
STOP_DIST = 1.0          # 近于1m：直接刹停/悬停
SLOW_DIST = 2.0          # 1~2m：龟速
MED_DIST  = 4.0          # 2~4m：中速

# 视野分5条竖向通道（基于默认 256x144 时比较合适；你可按分辨率调）
# 我们会根据实际宽度自动算分割点，所以这里不用死写50/100/...
BANDS = 5

# 只看上半部分（避免地面影响）
TOP_RATIO = 0.5

# 计算每条通道“最近障碍距离”的统计量：用 10th percentile（更稳，抗噪声）
P_NEAR = 10


def connect_takeoff(client: airsim.MultirotorClient):
    client.confirmConnection()
    print("✅ 连接成功")

    client.enableApiControl(True, vehicle_name=VEHICLE)
    client.armDisarm(True, vehicle_name=VEHICLE)
    print("✅ 已拿到控制权")

    client.takeoffAsync(vehicle_name=VEHICLE).join()
    client.moveToZAsync(CRUISE_Z, 2.0, vehicle_name=VEHICLE).join()
    client.hoverAsync(vehicle_name=VEHICLE).join()
    print(f"✅ 起飞完成，巡航高度 z={CRUISE_Z}")


def safe_shutdown(client: airsim.MultirotorClient):
    try:
        client.hoverAsync(vehicle_name=VEHICLE).join()
    except:
        pass
    try:
        # 软降到近地再 land，减少“砸地”
        client.moveToZAsync(-1.5, 1.0, vehicle_name=VEHICLE).join()
    except:
        pass
    try:
        client.landAsync(vehicle_name=VEHICLE).join()
        time.sleep(0.3)
    except:
        pass
    try:
        client.armDisarm(False, vehicle_name=VEHICLE)
        client.enableApiControl(False, vehicle_name=VEHICLE)
    except:
        pass
    cv2.destroyAllWindows()
    print("✅ 已降落停桨并归还控制权")


def get_depth_perspective_meters(client: airsim.MultirotorClient):
    """
    返回:
      depth: (H,W) float32，单位米（无效值会被处理）
    """
    req = airsim.ImageRequest(
        CAMERA,
        airsim.ImageType.DepthPerspective,
        pixels_as_float=True,
        compress=False
    )
    responses = client.simGetImages([req], vehicle_name=VEHICLE)
    if not responses or len(responses) == 0:
        return None

    r = responses[0]
    if r.width == 0 or r.height == 0:
        return None

    # AirSim: image_data_float 是长度 W*H 的 float list
    arr = np.array(r.image_data_float, dtype=np.float32)
    if arr.size != r.width * r.height:
        return None

    depth = arr.reshape(r.height, r.width)

    # 清理无效值：0、负数、inf、nan
    depth[~np.isfinite(depth)] = 1e6
    depth[depth <= 0] = 1e6

    # 给个上限，避免远处无穷大影响统计
    depth = np.clip(depth, 0.0, 100.0)
    return depth


def choose_band_by_depth(depth: np.ndarray):
    """
    把图像上半部分分成5条竖带，计算每条“近距离统计量”（10分位）
    返回:
      best_idx: 0..4  距离最大的通道（最安全）
      band_near: list[5] 每条通道的“近距离估计”（米）
      current_near: 中间通道近距离（米）
    """
    H, W = depth.shape
    top_h = int(H * TOP_RATIO)
    top = depth[:top_h, :]

    # 自动均分成BANDS条
    xs = [int(W * i / BANDS) for i in range(1, BANDS)]
    bands = np.hsplit(top, xs)

    band_near = []
    for b in bands:
        # 用 P_NEAR 分位数作为“这条通道最近障碍的稳健估计”
        v = float(np.percentile(b, P_NEAR))
        band_near.append(v)

    best_idx = int(np.argmax(band_near))
    current_near = band_near[BANDS // 2]  # 中间通道
    return best_idx, band_near, current_near


def yaw_delta_deg_for_band(idx: int):
    # 5条带 -> [-36, -18, 0, +18, +36]（可按需要加大）
    mapping = [-36.0, -18.0, 0.0, 18.0, 36.0]
    return mapping[idx]


def speed_from_distance(d: float):
    # d 越小越危险，速度越低
    if d < STOP_DIST:
        return 0.0
    if d < SLOW_DIST:
        return V_SLOW
    if d < MED_DIST:
        return V_MED
    return V_FAST


def main():
    client = airsim.MultirotorClient()
    connect_takeoff(client)

    print("按 q/esc 退出。DepthPerspective（米）避障启动。")

    while True:
        t0 = time.time()

        depth = get_depth_perspective_meters(client)
        if depth is None:
            print("⚠️ 拿不到 DepthPerspective。先试试 Scene 是否能拿到，或检查相机名/vehicle_name。")
            time.sleep(0.2)
            continue

        best_idx, band_near, current_near = choose_band_by_depth(depth)

        # 动态速度
        v = speed_from_distance(current_near)

        # 极近：先刹停（hover），避免“来不及”
        if current_near < STOP_DIST:
            client.hoverAsync(vehicle_name=VEHICLE).join()
            print(f"🛑 前方 {current_near:.2f}m，紧急悬停！ bands={['%.1f'%x for x in band_near]}")
            time.sleep(0.1)
            # 继续下一帧重新决策
            continue

        # 目标航向：朝最安全通道偏转
        pitch, roll, yaw_now = airsim.quaternion_to_euler_angles(
            client.simGetVehiclePose(vehicle_name=VEHICLE).orientation
        )
        yaw_target = yaw_now + math.radians(yaw_delta_deg_for_band(best_idx))

        vx = v * math.cos(yaw_target)
        vy = v * math.sin(yaw_target)

        # 让机头也转向（比“侧着飞”更不容易擦墙）
        yaw_mode = airsim.YawMode(True, math.degrees(yaw_target))

        client.moveByVelocityZAsync(
            vx, vy, CRUISE_Z,
            duration=DT,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=yaw_mode,
            vehicle_name=VEHICLE
        )

        # 可视化（简易）：把深度缩放成灰度显示（近亮远暗）
        # 注意：这是展示用，不参与决策
        vis = np.clip(depth, 0, 20)  # 只看0~20m
        vis = (1.0 - vis / 20.0) * 255.0
        vis = vis.astype(np.uint8)
        H, W = vis.shape
        top_h = int(H * TOP_RATIO)
        vis_bgr = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        # 画band分割线
        for i in range(1, BANDS):
            x = int(W * i / BANDS)
            cv2.line(vis_bgr, (x, 0), (x, top_h), (0, 255, 0), 1)

        # 标出best band
        x0 = int(W * best_idx / BANDS)
        x1 = int(W * (best_idx + 1) / BANDS)
        cv2.rectangle(vis_bgr, (x0, 0), (x1, top_h), (0, 0, 255), 2)

        cv2.putText(vis_bgr, f"front={current_near:.2f}m v={v:.2f}m/s",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.imshow("DepthPerspective (meters->vis)", vis_bgr)

        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord('q'), ord('x')]:
            break

        elapsed = time.time() - t0
        if elapsed < DT:
            time.sleep(DT - elapsed)

    safe_shutdown(client)


if __name__ == "__main__":
    main()