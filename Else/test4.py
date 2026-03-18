import cosysairsim as airsim
import time
import math

client = airsim.MultirotorClient()
client.confirmConnection()

drones = ["DroneA", "DroneB", "DroneC"]

# 目标区域（中心 + 半径）
cx, cy = 50.0, 50.0     # 区域中心（米）
r = 5.0                 # 区域半径（米）
target_z = -30.0        # 目标高度（NED，负数=离地更高）

# 区域内三个“编队点”（相对中心）
offsets = [
    (0.0, 0.0),                 # A：中心
    (r * math.cos(2*math.pi/3), r * math.sin(2*math.pi/3)),  # B：120°
    (r * math.cos(4*math.pi/3), r * math.sin(4*math.pi/3))   # C：240°
]

# 1) 逐台接管 + 解锁 + 起飞
for name in drones:
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

# 同时起飞
tasks = [client.takeoffAsync(vehicle_name=n) for n in drones]
[t.join() for t in tasks]

# 拉到同一高度（同时）
tasks = [client.moveToZAsync(target_z, 2.0, vehicle_name=n) for n in drones]
[t.join() for t in tasks]

print("✅ 三机起飞完成，开始飞向目标区域...")

# 2) 同时飞向目标区域内各自的编队点
tasks = []
for name, (ox, oy) in zip(drones, offsets):
    tx, ty = cx + ox, cy + oy
    tasks.append(client.moveToPositionAsync(tx, ty, target_z, 3.0, vehicle_name=name))
[t.join() for t in tasks]

print("✅ 三机已进入目标区域并完成编队")

# 3) 悬停保持（示例保持 10 秒）
tasks = [client.hoverAsync(vehicle_name=n) for n in drones]
[t.join() for t in tasks]
time.sleep(10)

# 4) 收尾：降落停桨归还
tasks = [client.landAsync(vehicle_name=n) for n in drones]
[t.join() for t in tasks]
time.sleep(0.3)

for name in drones:
    client.armDisarm(False, vehicle_name=name)
    client.enableApiControl(False, vehicle_name=name)

print("✅ 三机任务结束")