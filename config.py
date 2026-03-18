# 设置基站---目标物----任务类型---障碍物----禁飞区域等内容
# 统一设置为tag查找，然后转为actor读取详细信息
CONTROL_MODE = "swarm_cluster"
# CONTROL_MODE = "strict_formation"
# =========================
"""
1. 控制完每轮持续事件DT
2. sep 分离碰撞系数，这个值越大，斥力越强
3. 速度对齐系数 ALIGN 越大和周围邻居的速度越趋于一致
4. COH 聚集系数，越大，靠近邻居中心越多
5. mig 迁移
6. damp 阻尼系数 越大越抑制当前的速度

"""
# =========================
CLUSTER_NEIGHBOR_RADIUS = 8.0
CLUSTER_SAFE_DIST = 2.5
CLUSTER_MAX_SPEED = 8.0
CLUSTER_DT = 0.08
CLUSTER_TIMEOUT = 300.0
CLUSTER_SPEED = 5.0
CLUSTER_K_SEP = 2.8
CLUSTER_K_ALIGN = 0.25
CLUSTER_K_COH = 0.12
CLUSTER_K_MIG = 5.0
CLUSTER_K_DAMP = 0.05

CLUSTER_GOAL_XY_TOL = 10.0
CLUSTER_SPEED_TOL = 0.9
CLUSTER_STABLE_ROUNDS = 1
# 基站Tag设置
BASE_STATION_TAGS = ["JIZHAN1", "JIZHAN2"]

# 目标物Tag设置
TARGET_TAGS = ["Target"]

# 无人机名称，可拓展
UAV_NAMES = [
    "Drone1", "Drone2", "Drone3","Drone4", "Drone5", "Drone6", "Drone7", "Drone8", "Drone9"
]

# 无人机归属基站
BASE_STATION_ASSIGNMENT = {
    "JIZHAN1": ["Drone1", "Drone2", "Drone3","Drone8", "Drone9"],
    "JIZHAN2": ["Drone4", "Drone5", "Drone6", "Drone7"]
}

# =========================
# 地面初始部署偏移
# 以基站坐标为中心
# =========================
GROUND_DEPLOY_OFFSETS = {
    "JIZHAN1": [
        (-2.0, -2.0, -3.5),
        (-2.0,  0.0, -3.5),
        (-2.0,  2.0, -3.5),
        (-6.0,  2.0, -3.5),
        (-3.0,  4.0, -3.5),
    ],
    "JIZHAN2": [
        ( 2.0, -3.0, -3.5),
        ( 2.0, -1.0, -3.5),
        ( 2.0,  1.0, -3.5),
        ( 2.0,  3.0, -3.5),
    ]
}

# =========================
# 飞行初始参数
# TAKEOFF_HEIGHT 统一写正数，控制时再取负
# =========================
TAKEOFF_HEIGHT = 32.0
TAKEOFF_SPEED = 6.0
FORMATION_SPEED = 4.0

# 定义编排类型
STATION_FORMATION = {
    "JIZHAN1": "triangle_3",
    "JIZHAN2": "matrix_4"
}

# 定义具体的编排参数
FORMATION_PATTERNS = {
    "triangle_3": [
    ( 0.0,  0.0, 0.0),
    (-1.0, -1.0, 0.0),
    (-1.0,  1.0, 0.0),
    ],
    "matrix_4": [
        (-4.0, -4.0, 0.0),
        (-4.0,  4.0, 0.0),
        ( 4.0, -4.0, 0.0),
        ( 4.0,  4.0, 0.0),
    ]
}

# 定义两个编队找到任务的位置
STATION_ATTACK_CENTER_OFFSET = {
    "JIZHAN1": (-8.0,  0.0),
    "JIZHAN2": ( 8.0,  0.0),
}

# 定义六维属性   感知能力 计算能力 通信能力 能量容量 移动能力 任务载荷能力
"""
感知能力： 更适合做侦察 目标发现 图像获取 环境理解
计算能力： 更适合做推理 路径规划 数据处理
通信能力： 更适合做中继 组网节点 信息回传能力
能量容量： 续航能力 任务持续时间
移动能力： 机动性 速度 加速能力
任务载荷能力： 打击载荷 专用设备

"""
UAV_CAPABILITIES = {
    "Drone1": {
        "perception": 0.90,
        "compute": 0.60,
        "communication": 0.70,
        "energy_capacity": 0.80,
        "mobility": 0.80,
        "payload": 0.40,
        "payload_types": ["camera", "recon"]
    },
    "Drone2": {
        "perception": 0.80,
        "compute": 0.70,
        "communication": 0.80,
        "energy_capacity": 0.75,
        "mobility": 0.75,
        "payload": 0.45,
        "payload_types": ["camera", "relay"]
    },
    "Drone3": {
        "perception": 0.55,
        "compute": 0.55,
        "communication": 0.60,
        "energy_capacity": 0.90,
        "mobility": 0.70,
        "payload": 0.95,
        "payload_types": ["strike"]
    },
    "Drone4": {
        "perception": 0.88,
        "compute": 0.82,
        "communication": 0.72,
        "energy_capacity": 0.82,
        "mobility": 0.78,
        "payload": 0.35,
        "payload_types": ["camera", "recon"]
    },
    "Drone5": {
        "perception": 0.65,
        "compute": 0.92,
        "communication": 0.84,
        "energy_capacity": 0.76,
        "mobility": 0.62,
        "payload": 0.45,
        "payload_types": ["relay", "compute"]
    },
    "Drone6": {
        "perception": 0.55,
        "compute": 0.65,
        "communication": 0.95,
        "energy_capacity": 0.80,
        "mobility": 0.72,
        "payload": 0.40,
        "payload_types": ["relay"]
    },
    "Drone7": {
        "perception": 0.62,
        "compute": 0.58,
        "communication": 0.68,
        "energy_capacity": 0.92,
        "mobility": 0.92,
        "payload": 0.98,
        "payload_types": ["strike"]
    },
     "Drone8": {
        "perception": 0.62,
        "compute": 0.58,
        "communication": 0.68,
        "energy_capacity": 0.92,
        "mobility": 0.92,
        "payload": 0.98,
        "payload_types": ["strike"]
    },
     "Drone9": {
        "perception": 0.62,
        "compute": 0.58,
        "communication": 0.68,
        "energy_capacity": 0.92,
        "mobility": 0.92,
        "payload": 0.98,
        "payload_types": ["strike"]
    }
    
}


# 定义任务类型
TASK_LIBRARY = {
    "SEARCH": {
        "cn_name": "搜索任务",
        "description": "面向目标区域执行搜索、发现、确认与回传",
        "need_target": True,
        "allow_multi_uav": True,
        "allow_multi_station": False,
        "default_uav_count": 2,
        "stage_names": [
            "APPROACH_SEARCH_AREA",
            "SEARCH_SCAN",
            "TARGET_CONFIRM",
            "RESULT_REPORT"
        ],

        # 先做最小需求约束，后面再逐步扩展
        "min_perception": 0.70,
        "min_compute": 0.20,
        "min_communication": 0.40,
        "min_energy_capacity": 0.50,
        "required_payload_types": ["camera", "recon"]
    },

    "COOP_STRIKE": {
        "cn_name": "协同打击任务",
        "description": "面向目标执行协同接近、打击与撤离",
        "need_target": True,
        "allow_multi_uav": True,
        "allow_multi_station": True,
        "default_uav_count": 1,
        "stage_names": [
            "ASSEMBLE",
            "APPROACH_TARGET",
            "STRIKE_EXECUTE",
            "BDA_OR_WITHDRAW"
        ],

        "min_perception": 0.40,
        "min_compute": 0.30,
        "min_communication": 0.50,
        "min_energy_capacity": 0.60,
        "required_payload_types": ["strike"]
    }
}