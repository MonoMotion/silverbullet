import pybullet

import dataclasses
from enum import Enum, unique

@unique
class ConnectionMethod(Enum):
    DIRECT = pybullet.DIRECT
    GUI = pybullet.GUI
    SHARED_MEMORY = pybullet.SHARED_MEMORY

@dataclasses.dataclass(frozen=True)
class ConnectionInfo:
    is_connected: bool
    method: ConnectionMethod

