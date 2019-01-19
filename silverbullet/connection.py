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


class Connection(object):
    def __init__(self, mode=Mode.DIRECT):
        self.client = BulletClient(connection_mode=mode.to_bullet())

    def info(self) -> ConnectionInfo:
        result = self.client.getConnectionInfo()
        is_connected = bool(result['isConnected'])
        method = Mode(result['connectionMethod'])
        return ConnectionInfo(is_connected, method)

    def is_connected(self) -> bool:
        return self.info().is_connected

    def mode(self) -> Mode:
        return Mode(self.info().method)
