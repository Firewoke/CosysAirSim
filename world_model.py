from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# 转换函数
Vec3 = Tuple[float, float, float]


# 定义无人机的六维属性
@dataclass
class UAVCapability:
    perception: float
    compute: float
    communication: float
    energy_capacity: float
    mobility: float
    payload: float
    payload_types: List[str] = field(default_factory=list)

# 定义无人机的动态运行参数 能耗余量， 链路质量， 是否空闲以及当前执行的任务
@dataclass
class UAVRuntimeState:
    energy_remaining: float = 1.0
    comm_quality: float = 1.0
    is_alive: bool = True
    is_busy: bool = False
    current_task: Optional[str] = None

# 地面基站
@dataclass
class BaseStationModel:
    tag: str
    actor_name: str
    assigned_uavs: List[str] = field(default_factory=list)
    position_world: Optional[Vec3] = None

# 目标物 或者称为打击物
@dataclass
class TargetModel:
    tag: str
    actor_name: str
    status: str = "unknown"
    position_world: Optional[Vec3] = None

# 无人机的参数
@dataclass
class UAVModel:
    name: str
    station_tag: Optional[str] = None
    # 初始状态 静止
    status: str = "idle"
    deploy_slot_world: Optional[Vec3] = None
    current_world: Optional[Vec3] = None
    capability: Optional[UAVCapability] = None
    runtime: UAVRuntimeState = field(default_factory=UAVRuntimeState)

# 主类  激活其他的参数
@dataclass
class WorldModel:
    base_stations: Dict[str, BaseStationModel] = field(default_factory=dict)
    targets: Dict[str, TargetModel] = field(default_factory=dict)
    uavs: Dict[str, UAVModel] = field(default_factory=dict)

    def set_base_station_position(self, tag: str, pos: Vec3):
        if tag not in self.base_stations:
            raise KeyError(f"Base station not found: {tag}")
        self.base_stations[tag].position_world = pos

    def get_base_station_position(self, tag: str) -> Vec3:
        if tag not in self.base_stations:
            raise KeyError(f"Base station not found: {tag}")
        pos = self.base_stations[tag].position_world
        if pos is None:
            raise ValueError(f"Base station {tag} position_world is None")
        return pos

    def set_target_position(self, tag: str, pos: Vec3):
        if tag not in self.targets:
            raise KeyError(f"Target not found: {tag}")
        self.targets[tag].position_world = pos

    def get_target_position(self, tag: str) -> Vec3:
        if tag not in self.targets:
            raise KeyError(f"Target not found: {tag}")
        pos = self.targets[tag].position_world
        if pos is None:
            raise ValueError(f"Target {tag} position_world is None")
        return pos

    def set_uav_deploy_slot(self, uav_name: str, pos: Vec3):
        if uav_name not in self.uavs:
            raise KeyError(f"UAV not found: {uav_name}")
        self.uavs[uav_name].deploy_slot_world = pos

    def get_uav_deploy_slot(self, uav_name: str) -> Vec3:
        if uav_name not in self.uavs:
            raise KeyError(f"UAV not found: {uav_name}")
        pos = self.uavs[uav_name].deploy_slot_world
        if pos is None:
            raise ValueError(f"UAV {uav_name} deploy_slot_world is None")
        return pos

    def set_uav_current_world(self, uav_name: str, pos: Vec3):
        if uav_name not in self.uavs:
            raise KeyError(f"UAV not found: {uav_name}")
        self.uavs[uav_name].current_world = pos

    def set_uav_capability(self, uav_name: str, capability: UAVCapability):
        if uav_name not in self.uavs:
            raise KeyError(f"UAV not found: {uav_name}")
        self.uavs[uav_name].capability = capability

    def summary(self) -> str:
        lines = []
        lines.append("===== World Model Summary =====")
        lines.append("[BaseStations]")
        for tag, station in self.base_stations.items():
            lines.append(
                f"  - {tag}: actor={station.actor_name}, "
                f"assigned_uavs={station.assigned_uavs}, "
                f"position_world={station.position_world}"
            )

        lines.append("[Targets]")
        for tag, target in self.targets.items():
            lines.append(
                f"  - {tag}: actor={target.actor_name}, "
                f"status={target.status}, "
                f"position_world={target.position_world}"
            )

        lines.append("[UAVs]")
        for name, uav in self.uavs.items():
            cap_str = "None"
            if uav.capability is not None:
                cap_str = (
                    f"P={uav.capability.perception:.2f}, "
                    f"C={uav.capability.compute:.2f}, "
                    f"Comm={uav.capability.communication:.2f}, "
                    f"Ecap={uav.capability.energy_capacity:.2f}, "
                    f"M={uav.capability.mobility:.2f}, "
                    f"Pay={uav.capability.payload:.2f}, "
                    f"Types={uav.capability.payload_types}"
                )

            lines.append(
                f"  - {name}: "
                f"station={uav.station_tag}, "
                f"status={uav.status}, "
                f"deploy_slot_world={uav.deploy_slot_world}, "
                f"current_world={uav.current_world}, "
                f"runtime=(energy_remaining={uav.runtime.energy_remaining:.2f}, "
                f"comm_quality={uav.runtime.comm_quality:.2f}, "
                f"is_alive={uav.runtime.is_alive}, "
                f"is_busy={uav.runtime.is_busy}, "
                f"current_task={uav.runtime.current_task}), "
                f"capability={cap_str}"
            )

        return "\n".join(lines)