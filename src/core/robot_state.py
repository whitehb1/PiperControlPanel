from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass(slots=True)
class RobotState:
    timestamp: float = field(default_factory=time.time)
    joint_position: list[float] = field(default_factory=lambda: [0.0] * 6)
    gripper_position: float = 0.0
    is_enabled: bool = False
    error_code: int | None = None
    end_pose: list[float] | None = None

    def __post_init__(self) -> None:
        self.joint_position = [float(value) for value in self.joint_position]
        self.gripper_position = float(self.gripper_position)
        if len(self.joint_position) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(self.joint_position)}")
        if self.end_pose is not None:
            self.end_pose = [float(value) for value in self.end_pose]
            if len(self.end_pose) != 6:
                raise ValueError(f"Expected 6 end pose values, got {len(self.end_pose)}")

    def as_vector(self) -> list[float]:
        return [*self.joint_position, self.gripper_position]
