from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class UAVRuntimeState:
    perception_state: float
    compute_state: float
    communication_state: float
    energy_state: float
    mobility_state: float
    payload_state: float

    payload_types: List[str] = field(default_factory=list)
    is_online: bool = True
    is_available: bool = True
    current_task_id: Optional[str] = None

    @classmethod
    def from_capability(cls, cap):
        """
        根据静态能力画像初始化运行时状态
        第一版先直接等于静态值
        """
        return cls(
            perception_state=cap.perception,
            compute_state=cap.compute,
            communication_state=cap.communication,
            energy_state=cap.energy_capacity,
            mobility_state=cap.mobility,
            payload_state=cap.payload,
            payload_types=list(getattr(cap, "payload_types", [])),
            is_online=True,
            is_available=True,
            current_task_id=None,
        )

    def assign_task(self, task_id: str):
        self.current_task_id = task_id
        self.is_available = False

    def release_task(self):
        self.current_task_id = None
        self.is_available = True
