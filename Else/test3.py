import cosysairsim as airsim
import time
import numpy as np

NUM_UAV = 4  # 使用 4 台无人机
TARGET_Z = -30.0  # 目标飞行高度（负值表示向上）
DT = 0.1  # 控制周期（秒）
MAX_STEPS = 800  # 最大控制步数，防止无限循环卡住
Z_TOL = 0.5  # 到达目标高度的容差
MOVE_Z_TIMEOUT = 20.0  # 到达目标高度的最大等待时间（秒）
FORMATION_RAMP_STEPS = 200  # 编队展开时间（步）
USE_SAFE_ALTITUDE = True  # 是否先爬升到安全高度再水平编队
SAFE_Z = -50.0  # 安全高度（应高于场景障碍物最高点）

V_MAX = 2.0  # 最大平面速度（m/s）
K_ATT = 0.5  # 吸引势场系数
K_REP = 1.5  # 排斥势场系数
R_REP = 2.0  # 排斥作用半径（m）

FORMATION_SPACING = 3.0  # 编队间距（正方形边长）
TARGET_CENTER = np.array([25.0, 0.0], dtype=float)  # 集结目标中心点

ARRIVE_TOL = 0.5  # 到达阈值（m）
ARRIVE_STEPS = 20  # 连续到达步数，确保稳定

AUTO_LAND = True  # 是否自动降落
HOVER_TIME = 2.0  # 完成编队后的悬停时间
LAND_VELOCITY = 0.5  # 降落速度
PAUSE_ON_END = False  # 结束时是否暂停仿真

YAW_MODE = airsim.YawMode(is_rate=True, yaw_or_rate=0.0)  # 固定偏航，不随速度转向

origin_x = [0.0] * NUM_UAV  # 位置原点偏移（可用于坐标校正）
origin_y = [0.0] * NUM_UAV


def get_vehicle_name(i):
    # 根据索引拼接无人机名称
    return f"Drone{i + 1}"


def get_uav_pos(client, vehicle_name):
    # 读取单架无人机的真实位置（仿真真值）
    state = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
    x = state.position.x_val
    y = state.position.y_val
    i = int(vehicle_name[5:]) - 1
    x += origin_x[i]
    y += origin_y[i]
    return np.array([x, y], dtype=float)


def get_uav_pos3(client, vehicle_name):
    # 读取单架无人机的真实位置（x, y, z）
    state = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
    x = state.position.x_val
    y = state.position.y_val
    z = state.position.z_val
    i = int(vehicle_name[5:]) - 1
    x += origin_x[i]
    y += origin_y[i]
    return np.array([x, y, z], dtype=float)


def get_all_positions(client, num_uav):
    # 批量读取所有无人机位置，保证同一时刻使用同一组数据
    positions = []
    for i in range(num_uav):
        name = get_vehicle_name(i)
        positions.append(get_uav_pos(client, name))
    return positions


def takeoff_all(client, num_uav):
    # 并行起飞
    futures = []
    for i in range(num_uav):
        name = get_vehicle_name(i)
        client.enableApiControl(True, name)
        client.armDisarm(True, name)
        futures.append(client.takeoffAsync(vehicle_name=name))
    for f in futures:
        f.join()


def move_to_z_all(client, num_uav, target_z, velocity=1.0):
    # 所有无人机上升/下降到指定高度
    futures = []
    for i in range(num_uav):
        name = get_vehicle_name(i)
        futures.append(client.moveToZAsync(target_z, velocity, vehicle_name=name))
    start = time.time()
    last_report = -1
    while True:
        zs = []
        for i in range(num_uav):
            name = get_vehicle_name(i)
            pos = get_uav_pos3(client, name)
            zs.append(pos[2])
        if all(abs(z - target_z) <= Z_TOL for z in zs):
            break
        elapsed = time.time() - start
        if elapsed >= MOVE_Z_TIMEOUT:
            print("[WARN] move_to_z timeout. Z:", ", ".join(f"{z:.2f}" for z in zs))
            break
        sec = int(elapsed)
        if sec != last_report:
            last_report = sec
            print("[WAIT] move_to_z...", ", ".join(f"{z:.2f}" for z in zs))
        time.sleep(0.2)


def hover_all(client, num_uav):
    # 所有无人机悬停
    futures = []
    for i in range(num_uav):
        name = get_vehicle_name(i)
        futures.append(client.hoverAsync(vehicle_name=name))
    for f in futures:
        f.join()


def land_all(client, num_uav):
    # 所有无人机降落
    futures = []
    for i in range(num_uav):
        name = get_vehicle_name(i)
        futures.append(client.landAsync(vehicle_name=name))
    for f in futures:
        f.join()


def disarm_all(client, num_uav):
    # 解除武装并释放控制权
    for i in range(num_uav):
        name = get_vehicle_name(i)
        client.armDisarm(False, name)
        client.enableApiControl(False, name)


def clamp_velocity(v, v_max):
    # 速度限幅，避免控制量过大
    speed = np.linalg.norm(v)
    if speed > v_max:
        return v * (v_max / speed)
    return v


def square_offsets(num_uav, spacing):
    # 4 机正方形编队目标点偏移
    if num_uav != 4:
        raise ValueError("This demo expects exactly 4 UAVs.")
    s = spacing / 2.0
    return [
        np.array([-s, -s], dtype=float),
        np.array([-s, s], dtype=float),
        np.array([s, -s], dtype=float),
        np.array([s, s], dtype=float),
    ]


def assign_offsets_greedy(positions, offsets, center):
    # 将无人机与目标点一一匹配，减少整体移动距离（贪心）
    n = len(positions)
    targets = [center + off for off in offsets]
    assigned = [None] * n
    remaining_drones = set(range(n))
    remaining_targets = set(range(n))

    for _ in range(n):
        best_i = None
        best_t = None
        best_d = None
        for i in remaining_drones:
            pos_i = positions[i]
            for t in remaining_targets:
                d = np.linalg.norm(targets[t] - pos_i)
                if best_d is None or d < best_d:
                    best_d = d
                    best_i = i
                    best_t = t
        assigned[best_i] = offsets[best_t]
        remaining_drones.remove(best_i)
        remaining_targets.remove(best_t)

    return assigned


def build_target_positions(center, offsets):
    # 根据中心点与偏移生成最终目标点坐标
    return [center + off for off in offsets]


def compute_apf_velocity(positions, targets, v_max, k_att, k_rep, r_rep):
    # 计算 APF 控制速度：吸引到目标 + 排斥避障
    v_cmds = []
    for i, pos_i in enumerate(positions):
        # 吸引项：拉向目标点
        v_att = k_att * (targets[i] - pos_i)

        # 排斥项：避免无人机相互靠近
        v_rep = np.array([0.0, 0.0], dtype=float)
        for j, pos_j in enumerate(positions):
            if i == j:
                continue
            r = pos_i - pos_j
            d = np.linalg.norm(r)
            if d < 1e-6:
                continue
            if d < r_rep:
                v_rep += k_rep * (1.0 / d - 1.0 / r_rep) * (1.0 / (d * d)) * (r / d)

        v_cmds.append(clamp_velocity(v_att + v_rep, v_max))

    return v_cmds


def all_reached(positions, targets, tol):
    # 判断是否全部到达目标点
    for i, pos_i in enumerate(positions):
        if np.linalg.norm(targets[i] - pos_i) > tol:
            return False
    return True


def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    print("[INIT] Connected to AirSim.")
    print(f"[INIT] NUM_UAV={NUM_UAV} TARGET_Z={TARGET_Z} DT={DT}s MAX_STEPS={MAX_STEPS}")
    print(f"[INIT] TARGET_CENTER=({TARGET_CENTER[0]:.2f}, {TARGET_CENTER[1]:.2f}) SPACING={FORMATION_SPACING}")
    if USE_SAFE_ALTITUDE:
        print(f"[INIT] SAFE_Z={SAFE_Z}")

    if NUM_UAV < 1:
        raise ValueError("NUM_UAV must be >= 1")

    takeoff_all(client, NUM_UAV)
    print("[STEP] Takeoff complete.")
    if USE_SAFE_ALTITUDE:
        move_to_z_all(client, NUM_UAV, SAFE_Z, velocity=1.0)
        print("[STEP] Reached safe Z.")
        control_z = SAFE_Z
    else:
        move_to_z_all(client, NUM_UAV, TARGET_Z, velocity=1.0)
        print("[STEP] Reached target Z.")
        control_z = TARGET_Z

    positions = get_all_positions(client, NUM_UAV)
    offsets = square_offsets(NUM_UAV, FORMATION_SPACING)
    offsets = assign_offsets_greedy(positions, offsets, TARGET_CENTER)
    print("[STEP] Target positions assigned.")
    for i, off in enumerate(offsets):
        t = TARGET_CENTER + off
        print(f"[TARGET] Drone{i + 1}: x={t[0]:.2f}, y={t[1]:.2f}")

    stable_steps = 0
    for step in range(MAX_STEPS):
        positions = get_all_positions(client, NUM_UAV)
        # 编队偏移从 0 逐步展开到完整编队，减少早期交叉与混乱
        alpha = min(1.0, step / max(1, FORMATION_RAMP_STEPS))
        offsets_scaled = [alpha * off for off in offsets]
        targets = build_target_positions(TARGET_CENTER, offsets_scaled)
        v_cmds = compute_apf_velocity(
            positions,
            targets,
            V_MAX,
            K_ATT,
            K_REP,
            R_REP,
        )

        for i, v in enumerate(v_cmds):
            name = get_vehicle_name(i)
            client.moveByVelocityZAsync(
                v[0],
                v[1],
                control_z,
                DT,
                yaw_mode=YAW_MODE,
                vehicle_name=name,
            )

        if step % 10 == 0:
            dists = [np.linalg.norm(targets[i] - positions[i]) for i in range(NUM_UAV)]
            max_d = max(dists)
            print(f"[RUN] step={step} alpha={alpha:.2f} max_dist={max_d:.2f} stable={stable_steps}")

        if all_reached(positions, targets, ARRIVE_TOL) and alpha >= 1.0:
            stable_steps += 1
            if stable_steps >= ARRIVE_STEPS:
                print(f"[DONE] Formation reached and stable for {ARRIVE_STEPS} steps.")
                break
        else:
            stable_steps = 0

        time.sleep(DT)

    if stable_steps < ARRIVE_STEPS:
        print("[WARN] Max steps reached before stable formation.")

    if USE_SAFE_ALTITUDE:
        print("[STEP] Descending to target Z.")
        move_to_z_all(client, NUM_UAV, TARGET_Z, velocity=1.0)
        print("[STEP] Reached target Z.")

    hover_all(client, NUM_UAV)
    print("[STEP] Hover.")
    time.sleep(HOVER_TIME)
    if AUTO_LAND:
        land_all(client, NUM_UAV)
        disarm_all(client, NUM_UAV)
        print("[STEP] Landed and disarmed.")
    if PAUSE_ON_END:
        client.simPause(True)
        print("[STEP] Simulation paused.")


if __name__ == "__main__":
    main()
