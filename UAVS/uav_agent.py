from Tasks.task_types import TaskType, TaskTypeSpec


class UAVAgent:
    """
    单架无人机的任务适配代理
    负责回答：
    1. 能不能执行这个任务
    2. 如果能，适配分多高
    """

    def __init__(self, uav_name: str, capability, runtime_state):
        self.uav_name = uav_name
        self.capability = capability
        self.runtime_state = runtime_state

    def has_required_payload(self, spec: TaskTypeSpec) -> bool:
        required = spec.required_payload_types
        if not required:
            return True

        current_payloads = set(self.runtime_state.payload_types)
        for item in required:
            if item in current_payloads:
                return True
        return False

    def can_execute_task(self, spec: TaskTypeSpec) -> bool:
        if self.capability is None or self.runtime_state is None:
            return False

        if not self.runtime_state.is_online:
            return False

        if not self.runtime_state.is_available:
            return False

        if self.runtime_state.perception_state < spec.min_perception:
            return False

        if self.runtime_state.compute_state < spec.min_compute:
            return False

        if self.runtime_state.communication_state < spec.min_communication:
            return False

        if self.runtime_state.energy_state < spec.min_energy_capacity:
            return False

        if not self.has_required_payload(spec):
            return False

        return True

    def score_for_task(self, task, spec: TaskTypeSpec) -> float:
        r = self.runtime_state

        if task.task_type == TaskType.SEARCH:
            score = (
                0.40 * r.perception_state +
                0.20 * r.communication_state +
                0.20 * r.energy_state +
                0.20 * r.mobility_state
            )
        elif task.task_type == TaskType.COOP_STRIKE:
            score = (
                0.20 * r.communication_state +
                0.25 * r.energy_state +
                0.20 * r.mobility_state +
                0.25 * r.payload_state +
                0.10 * r.compute_state
            )
        else:
            score = (
                0.20 * r.perception_state +
                0.20 * r.compute_state +
                0.20 * r.communication_state +
                0.20 * r.energy_state +
                0.20 * r.mobility_state
            )

        if self.has_required_payload(spec):
            score += 0.1
        return score