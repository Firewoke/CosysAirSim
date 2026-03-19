from typing import Dict, List, Optional

from .task_request import TaskRequest
from .task_types import TaskType, get_task_type_spec
from BaseStations.base_station import BaseStationTaskAllocator
# 任务调度入口层
class TaskDispatcher:
    def __init__(self):
        self.tasks: Dict[str, TaskRequest] = {}
        self._counter = 0
        self.station_allocator = BaseStationTaskAllocator()

    def _next_task_id(self) -> str:
        self._counter += 1
        return f"TASK_{self._counter:03d}"

    def create_task(
        self,
        task_type: TaskType,
        source_station_tag: Optional[str] = None,
        target_tag: Optional[str] = None,
        target_position=None,
        note: str = ""
    ) -> TaskRequest:
        spec = get_task_type_spec(task_type)

        task = TaskRequest(
            task_id=self._next_task_id(),
            task_type=task_type,
            source_station_tag=source_station_tag,
            target_tag=target_tag,
            target_position=target_position,
            required_uav_count=spec.default_uav_count,
            allow_multi_station=spec.allow_multi_station,
            note=note
        )

        self.tasks[task.task_id] = task
        return task

    def get_task(self, task_id: str) -> TaskRequest:
        if task_id not in self.tasks:
            raise KeyError(f"Task not found: {task_id}")
        return self.tasks[task_id]

    def list_tasks(self) -> List[TaskRequest]:
        return list(self.tasks.values())

    def dispatch_task(self, task: TaskRequest, world):
        """
        - 按 source_station_tag 在本地基站内筛选
        - 返回候选排名
        - 选前 required_uav_count 架
        """
        if task.source_station_tag is None:
            raise ValueError(f"{task.task_id} has no source_station_tag")

        task.mark_dispatching()

        ranked_candidates = self.station_allocator.select_candidates(
            station_tag=task.source_station_tag,
            task=task,
            world=world
        )

        selected_uavs = [
            uav_name
            for uav_name, _ in ranked_candidates[:task.required_uav_count]
        ]

        task.bind_uavs(selected_uavs)

        ok = len(selected_uavs) >= task.required_uav_count

        # 分配成功后，把这些 UAV 标记为 busy
        if ok:
            for uav_name in selected_uavs:
                world.uavs[uav_name].runtime_state.assign_task(task.task_id)

        return {
            "ok": ok,
            "ranked_candidates": ranked_candidates,
            "selected_uavs": selected_uavs
        }