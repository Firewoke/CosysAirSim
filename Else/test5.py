import time
import cosysairsim as airsim

vn = "Drone1"
Z = -6
speed = 2.0
sample_dt = 0.05
max_time = 60.0
wp_tol = 0.3   # 航点到达阈值（m）

client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected!")

client.reset()
client.enableApiControl(True, vehicle_name=vn)
client.armDisarm(True, vehicle_name=vn)

client.takeoffAsync(vehicle_name=vn).join()
client.moveToZAsync(Z, 1, vehicle_name=vn).join()

client.simFlushPersistentMarkers()

points = [
    airsim.Vector3r(5, 0, Z),
    airsim.Vector3r(5, 5, Z),
    airsim.Vector3r(0, 5, Z),
    airsim.Vector3r(0, 0, Z),   
]
client.simPlotPoints(points, color_rgba=[0, 1, 0, 1], size=15, is_persistent=True)

# 发起路径任务
fut = client.moveOnPathAsync(points, speed, vehicle_name=vn)

trail = []
last_draw = 0
t0 = time.time()

wp_idx = 0
while True:
    kin = client.simGetGroundTruthKinematics(vehicle_name=vn)
    p = kin.position
    trail.append(airsim.Vector3r(p.x_val, p.y_val, p.z_val))

    # 刷新轨迹
    if len(trail) - last_draw >= 5:
        client.simPlotLineStrip(trail, color_rgba=[1, 0, 0, 1], thickness=5, is_persistent=True)
        last_draw = len(trail)

    # 打印确认在动
    if len(trail) % 20 == 0:
        print(f"[POS] x={p.x_val:.2f} y={p.y_val:.2f} z={p.z_val:.2f} wp={wp_idx}/{len(points)}")

    # 按顺序判断是否到达当前航点
    if wp_idx < len(points):
        wp = points[wp_idx]
        dx = p.x_val - wp.x_val
        dy = p.y_val - wp.y_val
        dz = p.z_val - wp.z_val
        if (dx*dx + dy*dy + dz*dz) ** 0.5 < wp_tol:
            print(f"[WP] reached {wp_idx}: ({wp.x_val},{wp.y_val},{wp.z_val})")
            wp_idx += 1

    # 走完所有航点才结束
    if wp_idx >= len(points):
        print("[INFO] Completed all waypoints.")
        break

    if time.time() - t0 > max_time:
        print("[WARN] Timeout waiting for path.")
        break

    time.sleep(sample_dt)

# 收尾
fut.join()
client.simPlotLineStrip(trail, color_rgba=[1, 0, 0, 1], thickness=5, is_persistent=True)
print("[DONE] trail points:", len(trail))

time.sleep(3)

client.goHomeAsync(vehicle_name=vn).join()
client.landAsync(vehicle_name=vn).join()
client.armDisarm(False, vehicle_name=vn)
client.enableApiControl(False, vehicle_name=vn)