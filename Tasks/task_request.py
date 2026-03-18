from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import time
# 任务实例层
from .task_types import TaskType, TaskPhase


@dataclass
class TaskRequest:
    task_id: str
    task_type: TaskType

    source_station_tag: Optional[str] = None
    target_tag: Optional[str] = None
    target_position: Optional[Tuple[float, float, float]] = None

    required_uav_count: int = 1
    allow_multi_station: bool = False

    phase: TaskPhase = TaskPhase.PENDING
    assigned_uavs: List[str] = field(default_factory=list)

    created_time: float = field(default_factory=time.time)
    started_time: Optional[float] = None
    finished_time: Optional[float] = None

    note: str = ""

    def mark_dispatching(self):
        self.phase = TaskPhase.DISPATCHING

    def mark_executing(self):
        if self.started_time is None:
            self.started_time = time.time()
        self.phase = TaskPhase.EXECUTING

    def mark_finished(self):
        self.phase = TaskPhase.FINISHED
        self.finished_time = time.time()

    def mark_failed(self):
        self.phase = TaskPhase.FAILED
        self.finished_time = time.time()

    def bind_uavs(self, uav_names: List[str]):
        self.assigned_uavs = list(uav_names)

    # 返航
    def mark_returning(self):
        self.phase = TaskPhase.RETURNING