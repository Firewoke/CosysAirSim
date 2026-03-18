# 测试简单飞行+控制降落
import cosysairsim as airsim
import time

name = "Drone1"

client = airsim.MultirotorClient()
client.confirmConnection()
print("连接成功")

client.enableApiControl(True, vehicle_name=name)
client.armDisarm(True, vehicle_name=name)
print("拿到无人机控制权")

print("准备起飞")
client.takeoffAsync(vehicle_name=name).join()

# ---- 第一段：飞到(0,0,-30)
print("飞行中：到 (0,0,-30)")
client.moveToPositionAsync(0, 0, -30, 5, vehicle_name=name).join()
print("到达第一目标点")

# ---- 悬停保持稳定
client.hoverAsync(vehicle_name=name).join()
time.sleep(0.5)

# ---- 得到当前无人机位置 全局坐标
state = client.getMultirotorState(vehicle_name=name)
p = state.kinematics_estimated.position
lock_x, lock_y, lock_z = p.x_val, p.y_val, p.z_val

# 用很小速度把位置“吸住”，避免漂移后直接开始第二段造成偏移
client.moveToPositionAsync(lock_x, lock_y, lock_z, 1.0, vehicle_name=name).join()
client.hoverAsync(vehicle_name=name).join()

print("悬停5s")
time.sleep(5)

print("再次执行飞行任务：到 (10,20,-30)")
client.moveToPositionAsync(10, 20, -30, 3, vehicle_name=name).join()

# ---- 控制下降速度，最后再落地
print("任务结束，开始着陆")


target_z_near_ground = -1.0
dt = 0.05  # 20Hz

# 获取当前高度
state = client.getMultirotorState(vehicle_name=name)
z = state.kinematics_estimated.position.z_val

while True:
    # 持续获取实时z轴高度
    state = client.getMultirotorState(vehicle_name=name)
    z = state.kinematics_estimated.position.z_val
    if z >= target_z_near_ground - 0.05:  # 接近目标就停
        break
    client.moveByVelocityZAsync(0, 0, target_z_near_ground, duration=dt, vehicle_name=name)
    time.sleep(dt)
    client.moveToZAsync
client.hoverAsync(vehicle_name=name).join()
client.landAsync(vehicle_name=name).join()

# 停稳一点再交出控制权
time.sleep(0.5)
client.armDisarm(False, vehicle_name=name)
client.enableApiControl(False, vehicle_name=name)

print("已降落并停桨，归还控制权")