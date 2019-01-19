import pybullet

import dataclasses
from enum import Enum, unique


@unique
class Mode(Enum):
    DIRECT = pybullet.DIRECT
    GUI = pybullet.GUI
    SHARED_MEMORY = pybullet.SHARED_MEMORY

    def to_bullet(self):
        return self.value

@dataclasses.dataclass(frozen=True)
class ConnectionInfo:
    is_connected: bool
    method: Mode

