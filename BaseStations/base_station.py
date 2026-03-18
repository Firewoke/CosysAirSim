from typing import List, Tuple

from Tasks.task_types import get_task_type_spec
from UAVS.uav_agent import UAVAgent
# 基站本地筛选层

class BaseStationTaskAllocator:
    """
    基站本地任务分配器
    只负责：
    1. 从本基站所属 UAV 中筛选候选
    2. 按任务适配分排序
    """

    def select_candidates(self, station_tag: str, task, world) -> List[Tuple[str, float]]:
        if station_tag not in world.base_stations:
            raise KeyError(f"Unknown station_tag: {station_tag}")

        station = world.base_stations[station_tag]
        spec = get_task_type_spec(task.task_type)

        ranked_candidates: List[Tuple[str, float]] = []

        for uav_name in station.assigned_uavs:
            if uav_name not in world.uavs:
                continue

            uav = world.uavs[uav_name]
            capability = getattr(uav, "capability", None)
            runtime_state = getattr(uav, "runtime_state", None)

            agent = UAVAgent(uav_name, capability, runtime_state)

            if agent.can_execute_task(spec):
                score = agent.score_for_task(task, spec)
                ranked_candidates.append((uav_name, score))

        ranked_candidates.sort(key=lambda x: x[1], reverse=True)
        return ranked_candidates