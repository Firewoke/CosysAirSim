import math
import time
from turtle import speed
import cosysairsim as airsim
import numpy as np
import os
import cv2
import threading
from UAVS.uav_state import UAVRuntimeState
from Tasks import TaskType, TaskDispatcher, list_task_type_specs
from config import (
    BASE_STATION_TAGS,
    TARGET_TAGS,
    UAV_NAMES,
    BASE_STATION_ASSIGNMENT,
    GROUND_DEPLOY_OFFSETS,
    TAKEOFF_HEIGHT,
    TAKEOFF_SPEED,
    UAV_CAPABILITIES,
    STATION_ATTACK_CENTER_OFFSET,
    CLUSTER_NEIGHBOR_RADIUS,
    CLUSTER_SAFE_DIST,
    CLUSTER_MAX_SPEED,
    CLUSTER_DT,
    CLUSTER_TIMEOUT,
    CLUSTER_K_SEP,
    CLUSTER_K_ALIGN,
    CLUSTER_K_COH,
    CLUSTER_K_MIG,
    CLUSTER_K_DAMP,
    CLUSTER_GOAL_XY_TOL,
    CLUSTER_SPEED_TOL,
    CLUSTER_STABLE_ROUNDS,
)
from scene_registry import SceneRegistry
from world_model import (
    WorldModel,
    BaseStationModel,
    TargetModel,
    UAVModel,
    UAVCapability,
)
from Tools.recording import (
    start_vehicle_camera_recording,
    stop_vehicle_camera_recording,
    start_multi_vehicle_recording,
    stop_multi_vehicle_recording,
)

# 查询无人机所属基站
def find_station_for_uav(uav_name: str):
    for station_tag, uav_list in BASE_STATION_ASSIGNMENT.items():
        if uav_name in uav_list:
            return station_tag
    return None


# 将二维度速度向量 约束在一个合理范围内
def clamp_2d(vx: float, vy: float, max_speed: float):
    speed = math.sqrt(vx * vx + vy * vy)
    if speed < 1e-6:
        return 0.0, 0.0
    if speed <= max_speed:
        return vx, vy
    scale = max_speed / speed
    return vx * scale, vy * scale



def build_ground_deploy_slots(world: WorldModel):
    """
    基于基站坐标，计算地面初始部署点

    offset 支持两种格式：
    1) (dx, dy)       -> z 直接使用基站自身 z
    2) (dx, dy, dz)   -> z = 基站 z + dz
    应该也可以获取其碰撞盒放上去
    """
    slots = {}
    # 找到相应的基站 
    for station_tag, station in world.base_stations.items():
        if station.position_world is None:
            raise RuntimeError(f"{station_tag} position_world is None")
        # 获取基站的坐标位置
        sx, sy, sz = station.position_world
        if station_tag not in GROUND_DEPLOY_OFFSETS:
            raise RuntimeError(f"{station_tag} 未配置 GROUND_DEPLOY_OFFSETS")

        offsets = GROUND_DEPLOY_OFFSETS[station_tag]
        uav_list = station.assigned_uavs

        if len(offsets) != len(uav_list):
            raise RuntimeError(f"{station_tag} ground offsets 数量与无人机数量不一致")
        # 针对每个无人机的部署去做偏移，计算出相应的位置信息
        for uav_name, offset in zip(uav_list, offsets):
            if len(offset) == 2:
                dx, dy = offset
                dz = 0.0
            elif len(offset) == 3:
                dx, dy, dz = offset
            else:
                raise RuntimeError(
                    f"{station_tag} 的 offset 格式错误: {offset}，"
                    f"应为 (dx, dy) 或 (dx, dy, dz)"
                )
            slots[uav_name] = (sx + dx, sy + dy, sz + dz)
    return slots


# 放置无人机在规定的基站
def spawn_uavs_at_deploy_slots(client, world: WorldModel):
    print("\n在规定位置创建无人机自主体")
    if hasattr(client, "listVehicles"):
        print("[INFO] Vehicles before spawn:", client.listVehicles())
    # 遍历所有的UAV和其自身的slot信息，然后去设置
    for uav_name, uav in world.uavs.items():
        if uav.deploy_slot_world is None:
            raise RuntimeError(f"{uav_name} deploy_slot_world is None")

        x, y, z = uav.deploy_slot_world
        pose = airsim.Pose(
            airsim.Vector3r(x, y, z),
            airsim.Quaternionr(0.0, 0.0, 0.0, 1.0)
        )

        ok = client.simAddVehicle(
            uav_name,
            "simpleflight",
            pose,
            ""
        )
        client.armDisarm(False, vehicle_name = uav_name)
    
        print(f"[SPAWN] name={uav_name}, pose=({x:.3f}, {y:.3f}, {z:.3f}), ok={ok}")
        if not ok:
            raise RuntimeError(f"simAddVehicle 失败: {uav_name}")

        world.set_uav_current_world(uav_name, (x, y, z))
    if hasattr(client, "listVehicles"):
        print("[INFO] Vehicles after spawn:", client.listVehicles())


# 初始化加载无人机六维度属性   在config已配置好初始状态 配置层 → 世界模型层 的映射
def load_uav_capabilities(world: WorldModel):
    print("\nLoading UAV capability profiles...")
    for uav_name, profile in UAV_CAPABILITIES.items():
        if uav_name not in world.uavs:
            continue
        cap = UAVCapability(
            perception=profile["perception"],
            compute=profile["compute"],
            communication=profile["communication"],
            energy_capacity=profile["energy_capacity"],
            mobility=profile["mobility"],
            payload=profile["payload"],
            payload_types=profile.get("payload_types", [])
        )
        world.set_uav_capability(uav_name, cap)
        print(f"[OK] capability loaded for {uav_name}")

# 根据每架 UAV 的静态能力 capability，生成它的运行时状态 runtime_state
def init_uav_runtime_states(world: WorldModel):
    print("\nInitializing UAV runtime states...")
    for uav_name, uav in world.uavs.items():
        if getattr(uav, "capability", None) is None:
            raise RuntimeError(f"{uav_name} capability is None")

        uav.runtime_state = UAVRuntimeState.from_capability(uav.capability)

        print(
            f"[OK] runtime state for {uav_name}: "
            f"P={uav.runtime_state.perception_state:.2f}, "
            f"C={uav.runtime_state.compute_state:.2f}, "
            f"Comm={uav.runtime_state.communication_state:.2f}, "
            f"E={uav.runtime_state.energy_state:.2f}, "
            f"M={uav.runtime_state.mobility_state:.2f}, "
            f"Payload={uav.runtime_state.payload_state:.2f}, "
            f"Types={uav.runtime_state.payload_types}"
        )


# 把一组筛选出来的无人机按照所属基站重新分组
def group_uavs_by_station(world: WorldModel, uav_names):
    station_map = {}
    for uav_name in uav_names:
        station_tag = world.uavs[uav_name].station_tag
        if station_tag not in station_map:
            station_map[station_tag] = []
        station_map[station_tag].append(uav_name)
    return station_map

# 把无人机姿态四元数转为yaw角，方便看机头朝向位置
def quaternion_to_yaw_deg(q) -> float:
    """
    由四元数直接计算 yaw（单位：度）
    公式与 AirSim 官方 utils.to_eularian_angles 中 yaw 部分一致
    """
    x = q.x_val
    y = q.y_val
    z = q.z_val
    w = q.w_val

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(t3, t4)
    return math.degrees(yaw_rad)

# 从 AirSim 里读取某架无人机的真实位置和真实速度，并把位置从“局部坐标”换算成“世界坐标”。
def get_uav_world_state(client, world: WorldModel, uav_name: str):
    """
    读取无人机真实状态，并转换成世界坐标
    返回：
        world_x, world_y, world_z, vx, vy, vz
    """
    # 根据uav_name，读取当前无人机的各种状态
    # 获取无人机的状态总量
    state = client.getMultirotorState(vehicle_name=uav_name)
    # 获取无人机的位置 基于无人机开始点
    pos = state.kinematics_estimated.position
    # 获取无人机的当前线性速度
    vel = state.kinematics_estimated.linear_velocity

    # AirSim 返回的是相对出生点的局部坐标
    home_x, home_y, home_z = world.get_uav_deploy_slot(uav_name)
    # 得到无人机的出生坐标，之后无论飞到哪里，都可以加上无人机的中间位移等等
    world_x = home_x + pos.x_val
    world_y = home_y + pos.y_val
    world_z = home_z + pos.z_val
    return world_x, world_y, world_z, vel.x_val, vel.y_val, vel.z_val

# 同步UAV的位置给世界模型中
def sync_uav_world_positions_from_sim(client, world: WorldModel, uav_names):
    """
    把仿真里的真实位置同步回 world model， 不同步速度
    """
    for uav_name in uav_names:
        wx, wy, wz, vx, vy, vz = get_uav_world_state(client, world, uav_name)
        world.set_uav_current_world(uav_name, (wx, wy, wz))

def hover_group(client, uav_names):
    futures = []
    for uav_name in uav_names:
        futures.append(client.hoverAsync(vehicle_name=uav_name))
    for f in futures:
        f.join()

# 控制集群无人机起飞函数
def arm_and_takeoff_uavs(client, world: WorldModel, uav_names, label: str = "selected UAVs"):
    if not uav_names:
        print(f"[WARN] No UAVs to take off for {label}.")
        return

    print(f"\nArming and taking off {label}: {uav_names}")

    target_z = TAKEOFF_HEIGHT if TAKEOFF_HEIGHT < 0 else -TAKEOFF_HEIGHT

    # 1) enable API + arm
    for uav_name in uav_names:
        client.enableApiControl(True, vehicle_name=uav_name)
        client.armDisarm(True, vehicle_name=uav_name)
        print(f"[OK] API+ARM: {uav_name}")

    # 2) 同时起飞
    takeoff_tasks = []
    for uav_name in uav_names:
        print(f"[TAKEOFF] {uav_name}")
        takeoff_tasks.append(client.takeoffAsync(vehicle_name=uav_name))

    for task in takeoff_tasks:
        task.join()

    # 3) 同时爬升到统一高度
    climb_tasks = []
    for uav_name in uav_names:
        print(f"[CLIMB] {uav_name} -> Z={target_z}")
        climb_tasks.append(
            client.moveToZAsync(
                target_z,
                TAKEOFF_SPEED,
                vehicle_name=uav_name
            )
        )

    for task in climb_tasks:
        task.join()

    # 4) 统一悬停
    hover_group(client, uav_names)

    # 5) 同步真实位置
    sync_uav_world_positions_from_sim(client, world, uav_names)

    for uav_name in uav_names:
        x, y, z = world.uavs[uav_name].current_world
        print(f"[HOVER] {uav_name}: world_x={x:.2f}, world_y={y:.2f}, world_z={z:.2f}")

# 起飞与悬停：全体 UAV
def arm_and_takeoff_all(client, world: WorldModel):
    arm_and_takeoff_uavs(
        client=client,
        world=world,
        uav_names=UAV_NAMES,
        label="all UAVs"
    )

# 起飞与悬停：指定 UAV 子集
def arm_and_takeoff_group(client, world: WorldModel, uav_names):
    arm_and_takeoff_uavs(
        client=client,
        world=world,
        uav_names=uav_names,
        label="selected UAVs"
    )


# =========================================================
# 状态同步
# =========================================================
def get_uav_world_state(client, world: WorldModel, uav_name: str):
    """
    读取无人机真实状态，并转换成世界坐标
    返回：
        world_x, world_y, world_z, vx, vy, vz
    """
    # 根据uav_name，读取当前无人机的各种状态
    # 获取无人机的状态总量
    state = client.getMultirotorState(vehicle_name=uav_name)
    # 获取无人机的位置 基于无人机开始点
    pos = state.kinematics_estimated.position
    # 获取无人机的当前线性速度
    vel = state.kinematics_estimated.linear_velocity

    # AirSim 返回的是相对出生点的局部坐标
    home_x, home_y, home_z = world.get_uav_deploy_slot(uav_name)
    # 得到无人机的出生坐标，之后无论飞到哪里，都可以加上无人机的中间位移等等
    world_x = home_x + pos.x_val
    world_y = home_y + pos.y_val
    world_z = home_z + pos.z_val
    return world_x, world_y, world_z, vel.x_val, vel.y_val, vel.z_val

# 同步UAV的位置给世界模型中
def sync_uav_world_positions_from_sim(client, world: WorldModel, uav_names):
    """
    把仿真里的真实位置同步回 world model， 不同步速度
    """
    for uav_name in uav_names:
        wx, wy, wz, vx, vy, vz = get_uav_world_state(client, world, uav_name)
        world.set_uav_current_world(uav_name, (wx, wy, wz))

"""
集群控制
让 station_goal_map 中列出的所有基站，其所属无人机集群，同时飞向各自目标点；
飞行过程中，每个集群内部保持一定聚合性、速度一致性，并避免彼此碰撞；
当所有集群都稳定到达目标点后，统一悬停结束。
"""
def swarm_all_stations_to_goals(
    client,
    world: WorldModel,
    station_goal_map,
    cruise_world_z_map,
    station_uavs_override=None,
    neighbor_radius: float = CLUSTER_NEIGHBOR_RADIUS,
    safe_dist: float = CLUSTER_SAFE_DIST,
    max_speed: float = CLUSTER_MAX_SPEED,
    dt: float = CLUSTER_DT,
    timeout_sec: float = CLUSTER_TIMEOUT,
    k_sep: float = CLUSTER_K_SEP,
    k_align: float = CLUSTER_K_ALIGN,
    k_coh: float = CLUSTER_K_COH,
    k_mig: float = CLUSTER_K_MIG,
    k_damp: float = CLUSTER_K_DAMP,
    goal_xy_tol: float = CLUSTER_GOAL_XY_TOL,
    speed_tol: float = CLUSTER_SPEED_TOL,
    stable_rounds: int = CLUSTER_STABLE_ROUNDS,
    accept_timeout_as_success: bool = True,
):
    """
    所有基站集群同时移动到各自目标点。

    每个基站内部独立做 boids 风格控制：
    - separation：避碰
    - cohesion   ：聚集
    - alignment  ：速度一致
    - migration  ：整体向目标迁移
    - damping    ：阻尼抑制，防止速度过大或震荡

    控制思路：
    1. 找出每个基站参与本轮控制的无人机
    2. 读取所有无人机真实状态
    3. 对每个基站分别计算集群中心与迁移方向
    4. 对站内每架无人机叠加避碰、聚集、对齐、迁移、阻尼速度
    5. 一次性给所有无人机下发速度命令
    6. 检查各个集群是否已“到位且稳定”
    """

    # =========================================================
    # 0) 确定本轮参与控制的基站与无人机
    # =========================================================
    station_tags = list(station_goal_map.keys())

    # 汇总本轮所有需要控制的无人机
    all_uav_names = []

    for station_tag in station_tags:
        if station_uavs_override is not None:
            uav_list = station_uavs_override.get(station_tag, [])
        else:
            uav_list = world.base_stations[station_tag].assigned_uavs

        all_uav_names.extend(uav_list)

    # 去重，防止某些异常配置导致同一无人机重复下发命令
    all_uav_names = list(dict.fromkeys(all_uav_names))

    # 如果本轮没有任何无人机，直接返回失败
    if not all_uav_names:
        print("[WARN] No UAVs available for swarm control.")
        return False

    # 记录每个基站已经连续稳定了多少轮
    stable_count_map = {station_tag: 0 for station_tag in station_tags}

    # 记录开始时间，用于超时退出
    start_time = time.time()

    # =========================================================
    # 主控制循环
    # =========================================================
    while True:
        # -----------------------------------------------------
        # 1) 先把仿真器里的真实位置同步回 world model
        # -----------------------------------------------------
        sync_uav_world_positions_from_sim(client, world, all_uav_names)

        # -----------------------------------------------------
        # 2) 读取所有参与控制无人机的真实状态
        #    这里只在 XY 平面做 boids，
        #    Z 方向由 moveByVelocityZAsync 固定到指定高度
        # -----------------------------------------------------
        global_state_map = {}

        for uav_name in all_uav_names:
            wx, wy, wz, vx, vy, vz = get_uav_world_state(client, world, uav_name)
            global_state_map[uav_name] = {
                "p": np.array([wx, wy], dtype=float),   # 世界坐标下的 XY 位置
                "v": np.array([vx, vy], dtype=float),   # 世界坐标下的 XY 速度
                "z": wz,                                # 当前世界 Z
                "vz": vz,                               # 当前 Z 速度
            }

        # -----------------------------------------------------
        # 3) 为所有无人机计算本轮速度命令
        # -----------------------------------------------------
        cmd_map = {}

        for station_tag in station_tags:
            # 取出该基站本轮参与控制的无人机
            if station_uavs_override is not None:
                uav_names = station_uavs_override.get(station_tag, [])
            else:
                uav_names = world.base_stations[station_tag].assigned_uavs

            # 如果该基站当前没有无人机参与，则跳过
            if not uav_names:
                print(f"[WARN] station={station_tag} has no UAVs, skip this round.")
                # 没有无人机时，直接认为它不阻塞整体结束判定
                stable_count_map[station_tag] = stable_rounds
                continue

            # 目标点只取 XY，Z 由 cruise_world_z_map 单独控制
            gx, gy, _ = station_goal_map[station_tag]

            # -------------------------------------------------
            # 3.1 计算该基站当前集群中心
            # -------------------------------------------------
            centroid = np.zeros(2, dtype=float)

            for uav_name in uav_names:
                centroid += global_state_map[uav_name]["p"]

            centroid /= len(uav_names)

            # -------------------------------------------------
            # 3.2 计算集群整体朝目标迁移的公共速度项
            # -------------------------------------------------
            goal_vec = np.array([gx, gy], dtype=float) - centroid
            goal_dist = np.linalg.norm(goal_vec)

            # 如果已经非常接近目标，则不给迁移速度
            if goal_dist < 1e-6:
                v_mig_global = np.zeros(2, dtype=float)
            else:
                # 方向单位化后乘迁移系数
                v_mig_global = k_mig * goal_vec / goal_dist

            # -------------------------------------------------
            # 3.3 对该基站中的每架无人机，分别计算 boids 速度
            # -------------------------------------------------
            for uav_name in uav_names:
                # 当前无人机 i 的位置和速度
                pos_i = global_state_map[uav_name]["p"]
                vel_i = global_state_map[uav_name]["v"]

                # 各项初始速度分量
                v_sep = np.zeros(2, dtype=float)    # 分离 / 避碰
                v_align = np.zeros(2, dtype=float)  # 对齐 / 速度一致
                v_coh = np.zeros(2, dtype=float)    # 聚集 / 靠近邻居中心

                # 邻居统计量
                neighbor_count = 0
                neighbor_center = np.zeros(2, dtype=float)
                neighbor_vel = np.zeros(2, dtype=float)

                # ---------------------------------------------
                # 遍历站内其他无人机，找邻居
                # ---------------------------------------------
                for other_name in uav_names:
                    if other_name == uav_name:
                        continue

                    pos_j = global_state_map[other_name]["p"]
                    vel_j = global_state_map[other_name]["v"]

                    # i 指向 j 的相对位移向量
                    rel = pos_j - pos_i
                    dist = np.linalg.norm(rel)

                    # 在邻居半径内，才计入邻居集合
                    if dist < neighbor_radius:
                        neighbor_count += 1
                        neighbor_center += pos_j
                        neighbor_vel += vel_j

                        # 如果距离太近，则加入避碰速度
                        if dist < safe_dist and dist > 1e-6:
                            # 距离越近，repel 越大
                            repel = (safe_dist - dist) / safe_dist
                            # 方向取反，远离邻居
                            v_sep += -k_sep * repel * (rel / dist)

                # ---------------------------------------------
                # 根据邻居集合计算聚集和速度一致项
                # ---------------------------------------------
                if neighbor_count > 0:
                    neighbor_center /= neighbor_count
                    neighbor_vel /= neighbor_count

                    # 聚集：朝邻居中心靠近
                    coh_vec = neighbor_center - pos_i
                    v_coh = k_coh * coh_vec

                    # 对齐：朝邻居平均速度靠拢
                    v_align = k_align * (neighbor_vel - vel_i)

                # 阻尼：抑制当前速度，减小震荡
                v_damp = -k_damp * vel_i

                # ---------------------------------------------
                # 合成最终速度
                # ---------------------------------------------
                v_cmd = v_sep + v_align + v_coh + v_mig_global + v_damp

                # 对速度做限幅，避免超过 max_speed
                vx_cmd, vy_cmd = clamp_2d(
                    float(v_cmd[0]),
                    float(v_cmd[1]),
                    max_speed
                )

                cmd_map[uav_name] = (vx_cmd, vy_cmd)

                print(
                    f"[SWARM CTRL] {station_tag}-{uav_name}: "
                    f"vx={vx_cmd:.2f}, vy={vy_cmd:.2f}, goal_dist={goal_dist:.2f}"
                )

        # -----------------------------------------------------
        # 4) 一次性给所有无人机下发速度命令
        # -----------------------------------------------------
        futures = []

        for uav_name in all_uav_names:
            # 某些极端情况下，如果该无人机没有生成控制命令，则跳过
            if uav_name not in cmd_map:
                print(f"[WARN] {uav_name} has no command this round, skip.")
                continue

            vx_cmd, vy_cmd = cmd_map[uav_name]

            # 该无人机所属基站
            station_tag = world.uavs[uav_name].station_tag

            # 目标巡航高度（世界坐标）
            cruise_world_z = cruise_world_z_map[station_tag]

            # AirSim moveByVelocityZAsync 需要的是“相对本机出生点”的 local z
            home_x, home_y, home_z = world.get_uav_deploy_slot(uav_name)
            local_target_z = cruise_world_z - home_z

            # 这里不显式刷新 yaw 目标，只保持 ForwardOnly
            # 避免 yaw 闭环和速度闭环互相冲突
            drivetrain = airsim.DrivetrainType.ForwardOnly
            yaw_mode = airsim.YawMode(False, 0)

            futures.append(
                client.moveByVelocityZAsync(
                    vx_cmd,
                    vy_cmd,
                    local_target_z,
                    dt,
                    drivetrain=drivetrain,
                    yaw_mode=yaw_mode,
                    vehicle_name=uav_name
                )
            )

        # 等待本轮所有无人机命令执行结束
        for f in futures:
            f.join()

        # -----------------------------------------------------
        # 5) 同步最新状态，并检查各集群是否已稳定到位
        # -----------------------------------------------------
       # -----------------------------------------------------
        # 5) 同步最新状态，并检查各集群是否已稳定到位
        # -----------------------------------------------------
        sync_uav_world_positions_from_sim(client, world, all_uav_names)

        all_station_finished = True

        for station_tag in station_tags:
            if station_uavs_override is not None:
                uav_names = station_uavs_override.get(station_tag, [])
            else:
                uav_names = world.base_stations[station_tag].assigned_uavs

            # 没有无人机的基站直接跳过
            if not uav_names:
                continue

            gx, gy, _ = station_goal_map[station_tag]

            centroid = np.zeros(2, dtype=float)     # 当前基站集群中心
            mean_speed_xy = 0.0                     # 当前基站集群平均水平速度
            max_goal_xy_err = 0.0                   # 该基站内单机最大目标误差

            for uav_name in uav_names:
                wx, wy, wz, vx, vy, vz = get_uav_world_state(client, world, uav_name)

                # 统计集群中心
                centroid += np.array([wx, wy], dtype=float)

                # 只统计水平速度，不把 vz 混进来
                speed_xy = math.hypot(vx, vy)
                mean_speed_xy += speed_xy

                # 统计单机到目标点的误差
                uav_goal_xy_err = math.hypot(gx - wx, gy - wy)
                max_goal_xy_err = max(max_goal_xy_err, uav_goal_xy_err)

                print(
                    f"[SWARM UAV STATUS] station={station_tag}, uav={uav_name}: "
                    f"pos=({wx:.2f},{wy:.2f},{wz:.2f}), "
                    f"vel=({vx:.2f},{vy:.2f},{vz:.2f}), "
                    f"speed_xy={speed_xy:.2f}, "
                    f"goal_xy_err={uav_goal_xy_err:.2f}"
                )

            centroid /= len(uav_names)
            mean_speed_xy /= len(uav_names)

            # 集群中心到目标点的 XY 误差
            centroid_goal_xy_err = np.linalg.norm(
                np.array([gx, gy], dtype=float) - centroid
            )

            # 判定“到位且稳定”
            # 这里我保留你原来的“中心点到位”思路，
            # 但额外要求单机最大误差也不能太大，避免一架到了另一架还很远。
            ok = (
                centroid_goal_xy_err <= goal_xy_tol
                and max_goal_xy_err <= goal_xy_tol * 1.5
                and mean_speed_xy <= speed_tol
            )

            print(
                f"[SWARM STATUS] station={station_tag}: "
                f"centroid_goal_xy_err={centroid_goal_xy_err:.2f}, "
                f"max_goal_xy_err={max_goal_xy_err:.2f}, "
                f"mean_speed_xy={mean_speed_xy:.2f}, ok={ok}"
            )

            if ok:
                stable_count_map[station_tag] += 1
            else:
                stable_count_map[station_tag] = 0

            print(
                f"[SWARM STABLE] station={station_tag}: "
                f"{stable_count_map[station_tag]}/{stable_rounds}"
            )

            if stable_count_map[station_tag] < stable_rounds:
                all_station_finished = False
        # -----------------------------------------------------
        # 6) 如果所有集群都稳定到位，则结束
        # -----------------------------------------------------
        if all_station_finished:
            hover_group(client, all_uav_names)
            sync_uav_world_positions_from_sim(client, world, all_uav_names)
            print("[OK] All swarm clusters confirmed.")
            return True

        # -----------------------------------------------------
        # 7) 超时退出
        # -----------------------------------------------------
        if time.time() - start_time > timeout_sec:
            hover_group(client, all_uav_names)
            sync_uav_world_positions_from_sim(client, world, all_uav_names)

            if accept_timeout_as_success:
                print("[WARN] Swarm timeout, but continue task flow as preview-success.")
                return True
            else:
                print("[WARN] Swarm timeout, task not truly converged.")
                return False

def release_task_uavs(world: WorldModel, task):
    for uav_name in task.assigned_uavs:
        if uav_name not in world.uavs:
            continue

        uav = world.uavs[uav_name]
        if hasattr(uav, "runtime_state") and uav.runtime_state is not None:
            uav.runtime_state.release_task()

def execute_return_to_base(client, world: WorldModel, task):
    print("\n==============================")
    print(f" Return Task: {task.task_id}")
    print("==============================")

    selected_uavs = task.assigned_uavs
    if not selected_uavs:
        print(f"[WARN] {task.task_id} has no assigned UAVs for return.")
        return False

    # 标记任务进入返航阶段
    task.mark_returning()

    # =========================================================
    # 1) 先回到各自 deploy slot 正上方的统一高度
    #    这里不能让多机都回同一个“基站中心点”，
    #    否则同一基站的多架无人机会发生位置冲突。
    #    因此返回目标应为每架无人机各自的 deploy slot。
    # =========================================================
    target_z = TAKEOFF_HEIGHT if TAKEOFF_HEIGHT < 0 else -TAKEOFF_HEIGHT

    move_tasks = []
    for uav_name in selected_uavs:
        # 对于动态生成的 UAV，
        # moveToPositionAsync(0, 0, target_z) 的局部原点就是自己的 deploy slot
        print(f"[RETURN SLOT] {uav_name} -> local_home=(0, 0, {target_z})")
        move_tasks.append(
            client.moveToPositionAsync(
                0.0,
                0.0,
                target_z,
                TAKEOFF_SPEED,
                vehicle_name=uav_name
            )
        )

    for task_future in move_tasks:
        task_future.join()

    # 回到部署位上方后，先统一悬停并同步位置
    hover_group(client, selected_uavs)
    sync_uav_world_positions_from_sim(client, world, selected_uavs)

    # =========================================================
    # 2) 执行降落
    # =========================================================
    land_tasks = []
    for uav_name in selected_uavs:
        print(f"[LAND] {uav_name}")
        land_tasks.append(client.landAsync(vehicle_name=uav_name))

    for task_future in land_tasks:
        task_future.join()

    # 降落完成后再次同步
    sync_uav_world_positions_from_sim(client, world, selected_uavs)

    # =========================================================
    # 3) 解除武装 + 关闭 API 控制权
    #    并将 world 中的位置账本矫正回 deploy slot
    # =========================================================
    for uav_name in selected_uavs:
        client.armDisarm(False, vehicle_name=uav_name)
        client.enableApiControl(False, vehicle_name=uav_name)

        home_x, home_y, home_z = world.get_uav_deploy_slot(uav_name)
        world.set_uav_current_world(uav_name, (home_x, home_y, home_z))

        print(
            f"[LANDED] {uav_name} -> "
            f"deploy_slot_world=({home_x:.2f}, {home_y:.2f}, {home_z:.2f})"
        )

    print("[OK] Return, landing, disarm, and API release finished.")
    return True

def execute_search_task(client, world: WorldModel, task):
    print("\n==============================")
    print(f" Execute Task: {task.task_id} [SEARCH]")
    print("==============================")

    selected_uavs = task.assigned_uavs
    if not selected_uavs:
        print(f"[WARN] {task.task_id} has no assigned UAVs.")
        task.mark_failed()
        return False

    task.mark_executing()

    # 1. 只起飞被选中的 UAV
    arm_and_takeoff_group(client, world, selected_uavs)

    # 2. 按基站分组
    station_uavs_map = group_uavs_by_station(world, selected_uavs)

    # 3. 构建搜索目标点（第一版：直接飞到目标点附近）
    tx, ty, tz = task.target_position
    station_goal_map = {}
    cruise_world_z_map = {}

    for station_tag, uav_names in station_uavs_map.items():
        first_uav = uav_names[0]
        _, _, current_z = world.uavs[first_uav].current_world

        station_goal_map[station_tag] = (tx, ty, current_z)
        cruise_world_z_map[station_tag] = current_z

        print(f"[SEARCH GOAL] station={station_tag}, goal={station_goal_map[station_tag]}")

    # 4. 只让被选中的 UAV 做 swarm 接近
    ok = swarm_all_stations_to_goals(
        client=client,
        world=world,
        station_goal_map=station_goal_map,
        cruise_world_z_map=cruise_world_z_map,
        station_uavs_override=station_uavs_map
    )

     # 5. 返航阶段
    return_ok = execute_return_to_base(client, world, task)

    # 6. 释放资源
    release_task_uavs(world, task)

    # 7. 标记任务结束
    if ok and return_ok:
        task.mark_finished()
    else:
        task.mark_failed()

    return ok and return_ok


def execute_coop_strike_task(client, world: WorldModel, task):
    print("\n==============================")
    print(f" Execute Task: {task.task_id} [COOP_STRIKE]")
    print("==============================")

    selected_uavs = task.assigned_uavs
    if not selected_uavs:
        print(f"[WARN] {task.task_id} has no assigned UAVs.")
        task.mark_failed()
        return False

    task.mark_executing()

    # 1. 只起飞被选中的 UAV
    arm_and_takeoff_group(client, world, selected_uavs)

    # 2. 按基站分组
    station_uavs_map = group_uavs_by_station(world, selected_uavs)

    # 3. 构建打击接近点
    tx, ty, tz = task.target_position
    station_goal_map = {}
    cruise_world_z_map = {}

    for station_tag, uav_names in station_uavs_map.items():
        offx, offy = STATION_ATTACK_CENTER_OFFSET[station_tag]
        first_uav = uav_names[0]
        _, _, current_z = world.uavs[first_uav].current_world

        station_goal_map[station_tag] = (tx + offx, ty + offy, current_z)
        cruise_world_z_map[station_tag] = current_z

        print(f"[STRIKE GOAL] station={station_tag}, goal={station_goal_map[station_tag]}")

    # 4. 只让被选中的 UAV 做 swarm 接近
    ok = swarm_all_stations_to_goals(
        client=client,
        world=world,
        station_goal_map=station_goal_map,
        cruise_world_z_map=cruise_world_z_map,
        station_uavs_override=station_uavs_map
    )

    # 5. 返航阶段
    return_ok = execute_return_to_base(client, world, task)

    # 6. 释放资源
    release_task_uavs(world, task)

    # 7. 标记任务结束
    if ok and return_ok:
        task.mark_finished()
    else:
        task.mark_failed()

    return ok and return_ok

# =========================================================
# 当前任务入口：只保留 swarm cluster
# =========================================================
def execute_coop_strike_demo(client, world: WorldModel):
    """
    当前阶段只做集群控制：
    1. 全部起飞到固定高度
    2. 不做三角形/矩阵等严格编排
    3. 每个基站内部用集群一致性控制
    4. 两个基站同时朝目标附近的攻击接近点移动
    """
    print("\n==============================")
    print(" Start Cooperative Strike Demo [SWARM ONLY]")
    print("==============================")

    target_pos = world.get_target_position("Target")
    print(f"[TARGET] {target_pos}")

    # 1. 全部起飞
    arm_and_takeoff_all(client, world)
    sync_uav_world_positions_from_sim(client, world, UAV_NAMES)


    # 2. 构建两个基站的目标点
    tx, ty, tz = target_pos
    station_goal_map = {}
    cruise_world_z_map = {}

    print("\n[PHASE] Swarm clusters approach target simultaneously...")

    for station_tag in BASE_STATION_TAGS:
        station_uavs = world.base_stations[station_tag].assigned_uavs
        offx, offy = STATION_ATTACK_CENTER_OFFSET[station_tag]

        first_uav = station_uavs[0]
        _, _, current_z = world.uavs[first_uav].current_world

        station_goal_map[station_tag] = (tx + offx, ty + offy, current_z)
        cruise_world_z_map[station_tag] = current_z

        print(f"[SWARM GOAL] station={station_tag}, goal={station_goal_map[station_tag]}")

    # 3. 两个集群同时移动
    ok = swarm_all_stations_to_goals(
        client=client,
        world=world,
        station_goal_map=station_goal_map,
        cruise_world_z_map=cruise_world_z_map,
    )

    if not ok:
        print("[WARN] Swarm clusters did not fully converge before timeout.")

    hover_group(client, UAV_NAMES)
    sync_uav_world_positions_from_sim(client, world, UAV_NAMES)

    print("\n[OK] Cooperative strike demo [SWARM ONLY] finished.")
    print(world.summary())

    print("[OK] All swarm clusters confirmed.")
    return True



# =========================================================
# main 
# =========================================================
def main():
    print("Connecting to Cosys-AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[OK] Connected.\n")

    registry = SceneRegistry(client)
    world = WorldModel()

    # 1. 注册基站
    print("Registering base stations...")
    station_map = registry.register_tags(BASE_STATION_TAGS)
    for tag, actor_name in station_map.items():
        assigned_uavs = BASE_STATION_ASSIGNMENT.get(tag, [])

        world.base_stations[tag] = BaseStationModel(
            tag=tag,
            actor_name=actor_name,
            assigned_uavs=assigned_uavs
        )

        pos = registry.get_actor_world_position(actor_name)
        print(f"[DEBUG] BaseStation {tag} actor={actor_name} pos={pos}")
        world.set_base_station_position(tag, pos)
        print(f"[OK] Found BaseStation: {tag} -> {actor_name}")

    print()

    # 2. 注册目标
    print("Registering targets...")
    target_map = registry.register_tags(TARGET_TAGS)
    for tag, actor_name in target_map.items():
        world.targets[tag] = TargetModel(
            tag=tag,
            actor_name=actor_name
        )

        pos = registry.get_actor_world_position(actor_name)
        print(f"[DEBUG] Target {tag} actor={actor_name} pos={pos}")
        world.set_target_position(tag, pos)
        print(f"[OK] Found Target: {tag} -> {actor_name}")

    print()

    # 3. 注册无人机
    print("Registering UAVs...")
    for uav_name in UAV_NAMES:
        station_tag = find_station_for_uav(uav_name)
        world.uavs[uav_name] = UAVModel(
            name=uav_name,
            station_tag=station_tag
        )
        print(f"[OK] UAV Registered: {uav_name}, station={station_tag}")

    print()

    # 4. 计算地面部署槽位
    print("Planning ground deploy slots...")
    ground_slots = build_ground_deploy_slots(world)
    for uav_name, slot_world in ground_slots.items():
        world.set_uav_deploy_slot(uav_name, slot_world)
        print(f"[OK] {uav_name} deploy_slot_world = {slot_world}")

   

    # 5. 动态生成无人机
    spawn_uavs_at_deploy_slots(client, world)

    # 6. 加载属性
    load_uav_capabilities(world)
    

    init_uav_runtime_states(world)
    print("\n[OK] World model initialized.")
    print(world.summary())

    print("\n[INFO] Available task types:")
    for spec in list_task_type_specs():
        print(
            f"  - {spec.task_type.value}: "
            f"{spec.cn_name}, "
            f"default_uav_count={spec.default_uav_count}, "
            f"stages={spec.stage_names}"
        )

    dispatcher = TaskDispatcher()

    # =========================
    # 创建两个测试任务
    # =========================
    search_task = dispatcher.create_task(
        task_type=TaskType.SEARCH,
        source_station_tag="JIZHAN1",
        target_tag="Target",
        target_position=world.get_target_position("Target"),
        note="测试搜索任务"
    )

    strike_task = dispatcher.create_task(
        task_type=TaskType.COOP_STRIKE,
        source_station_tag="JIZHAN2",
        target_tag="Target",
        target_position=world.get_target_position("Target"),
        note="测试协同打击任务"
    )

    print("\n[INFO] Created tasks:")
    for task in dispatcher.list_tasks():
        print(task)

    # =========================
    # 基站本地任务分配测试
    # =========================
    print("\n[DISPATCH TEST] SEARCH @ JIZHAN1")
    search_result = dispatcher.dispatch_task(search_task, world)
    print("ranked_candidates =", search_result["ranked_candidates"])
    print("selected_uavs     =", search_result["selected_uavs"])
    print("ok                =", search_result["ok"])

    print("\n[DISPATCH TEST] COOP_STRIKE @ JIZHAN2")
    strike_result = dispatcher.dispatch_task(strike_task, world)
    print("ranked_candidates =", strike_result["ranked_candidates"])
    print("selected_uavs     =", strike_result["selected_uavs"])
    print("ok                =", strike_result["ok"])


    # 7. 执行当前任务
    print("\n==============================")
    print(" START TASK EXECUTION CHAIN")
    print("==============================")

    # 先执行搜索任务
    if search_result["ok"]:
        print(f"[RUN] {search_task.task_id} -> {search_task.assigned_uavs}")
        execute_search_task(client, world, search_task)
    else:
        print(f"[SKIP] {search_task.task_id} dispatch not satisfied.")

    # 再执行协同打击任务
    if strike_result["ok"]:
        print(f"[RUN] {strike_task.task_id} -> {strike_task.assigned_uavs}")
        execute_coop_strike_task(client, world, strike_task)
    else:
        print(f"[SKIP] {strike_task.task_id} dispatch not satisfied.")

if __name__ == "__main__":
    main()
