from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List
from config import TASK_LIBRARY

# 放任务类型定义 任务类型语义层
class TaskType(str, Enum):
    SEARCH = "SEARCH"
    COOP_STRIKE = "COOP_STRIKE"


class TaskPhase(str, Enum):
    PENDING = "PENDING"
    DISPATCHING = "DISPATCHING"
    EXECUTING = "EXECUTING"
    FINISHED = "FINISHED"
    RETURNING = "RETURNING"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class TaskTypeSpec:
    task_type: TaskType
    cn_name: str
    description: str

    need_target: bool = True
    allow_multi_uav: bool = True
    allow_multi_station: bool = False
    default_uav_count: int = 1
    stage_names: List[str] = field(default_factory=list)

    min_perception: float = 0.0
    min_compute: float = 0.0
    min_communication: float = 0.0
    min_energy_capacity: float = 0.0
    required_payload_types: List[str] = field(default_factory=list)


def _build_task_type_library() -> Dict[TaskType, TaskTypeSpec]:
    lib: Dict[TaskType, TaskTypeSpec] = {}

    for task_name, cfg in TASK_LIBRARY.items():
        task_type = TaskType(task_name)

        lib[task_type] = TaskTypeSpec(
            task_type=task_type,
            cn_name=cfg["cn_name"],
            description=cfg["description"],
            need_target=cfg.get("need_target", True),
            allow_multi_uav=cfg.get("allow_multi_uav", True),
            allow_multi_station=cfg.get("allow_multi_station", False),
            default_uav_count=cfg.get("default_uav_count", 1),
            stage_names=cfg.get("stage_names", []),
            min_perception=cfg.get("min_perception", 0.0),
            min_compute=cfg.get("min_compute", 0.0),
            min_communication=cfg.get("min_communication", 0.0),
            min_energy_capacity=cfg.get("min_energy_capacity", 0.0),
            required_payload_types=cfg.get("required_payload_types", []),
        )

    return lib


TASK_TYPE_LIBRARY = _build_task_type_library()


def get_task_type_spec(task_type: TaskType) -> TaskTypeSpec:
    if task_type not in TASK_TYPE_LIBRARY:
        raise KeyError(f"Unknown TaskType: {task_type}")
    return TASK_TYPE_LIBRARY[task_type]


def list_task_type_specs():
    return list(TASK_TYPE_LIBRARY.values())