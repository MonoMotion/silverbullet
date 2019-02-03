import dataclasses
import pybullet
from pybullet_utils import bullet_client
import numpy as np
from .scene import Scene

import functools
from typing import Optional, Dict, Tuple
import re


@dataclasses.dataclass
class Pose:
    vector: np.ndarray
    quaternion: np.ndarray

    def dot(self, pose: 'Pose'):
        vector, quaternion = pybullet.multiplyTransforms(
            self.vector, self.quaternion, pose.vector, pose.quaternion)
        return Pose(vector, quaternion)


@dataclasses.dataclass
class JointState:
    position: float
    velocity: float
    applied_torque: float


@dataclasses.dataclass
class LinkState:
    pose: Pose
    linear_velocity: Optional[np.ndarray]
    angular_velocity: Optional[np.ndarray]


@dataclasses.dataclass
class ClientWithBody:
    client: bullet_client.BulletClient
    body_id: int

    def __getattr__(self, name: str):
        method = getattr(self.client, name)
        return functools.partial(method, bodyUniqueId=self.body_id)


@dataclasses.dataclass
class DynamicsInfo:
    mass: float
    lateral_friction: float
    local_inertia_diagonal: Tuple[float, float, float]
    local_inertial_position: Tuple[float, float, float]
    local_inertial_orientation: Tuple[float, float, float, float]
    restitution: float
    rolling_friction: float
    spinning_friction: float
    contact_damping: Optional[float] = None
    contact_stiffness: Optional[float] = None


@dataclasses.dataclass
class Robot:
    body_id: int
    scene: dataclasses.InitVar[Scene]

    joints: Dict[str, int] = dataclasses.field(init=False)
    links: Dict[str, int] = dataclasses.field(init=False)
    client: ClientWithBody = dataclasses.field(init=False)
    root_link: str = dataclasses.field(init=False)

    def __post_init__(self, scene: Scene):
        self.client = ClientWithBody(scene._conn.client, self.body_id)
        nr_joints = self.client.getNumJoints()
        self.links = {}
        self.joints = {}

        root, _ = self.client.getBodyInfo()
        root_name = root.decode()
        self.links[root_name] = -1
        self.root_link = root_name

        for idx in range(nr_joints):
            result = self.client.getJointInfo(jointIndex=idx)
            joint_name = result[1].decode()
            link_name = result[12].decode()
            self.links[link_name] = idx
            self.joints[joint_name] = idx

    def joint_state(self, name: str) -> JointState:
        joint_id = self.joints[name]
        pos, vel, _, torque = self.client.getJointState(jointIndex=joint_id)
        return JointState(pos, vel, torque)

    def link_state(self, name: str, compute_velocity=False) -> LinkState:
        link_id = self.links[name]
        if link_id == -1:
            pos, ori = self.client.getBasePositionAndOrientation()
            if compute_velocity:
                a_vel, l_vel = self.client.getBaseVelocity()

        else:
            ret = self.client.getLinkState(
                linkIndex=link_id, computeLinkVelocity=compute_velocity)

            if compute_velocity:
                pos, ori, _, _, _, _, l_vel, a_vel = ret
            else:
                pos, ori, _, _, _, _ = ret

        if not compute_velocity:
            a_vel = None
            l_vel = None

        pose = Pose(np.array(pos), np.array(ori))
        return LinkState(pose, l_vel, a_vel)

    def set_joint_position(self, name: str, target: float, *,
                           kp: float = 0.1, kd: float = 1.0, force: float = 100000.0):
        joint_id = self.joints[name]
        self.client.setJointMotorControl2(
            jointIndex=joint_id,
            controlMode=pybullet.POSITION_CONTROL,
            targetPosition=target,
            positionGain=kp,
            velocityGain=kd,
            force=force)

    def set_joint_velocity(self, name: str, target: float, *, force: float = 100000.0):
        joint_id = self.joints[name]
        self.client.setJointMotorControl2(
            jointIndex=joint_id, controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocity=target, force=force)

    def set_joint_torque(self, name: str, force: float):
        joint_id = self.joints[name]
        self.client.setJointMotorControl2(
            jointIndex=joint_id, controlMode=pybullet.TORQUE_CONTROL, force=force)

    def bring_on_the_ground(self, padding: float = 0):
        h = min(self.link_state(name).pose.vector[2]
                for name in self.links.keys())
        if h > 0:
            raise RuntimeError("Robot is already on the ground")
        self.client.resetBasePositionAndOrientation(
            posObj=[0, 0, -h + padding], ornObj=[0, 0, 0, 1])

    def dynamics_info(self, name: str = None) -> DynamicsInfo:
        if name is None:
            link_id = -1
        else:
            link_id = self.links[name]

        ret = self.client.getDynamicsInfo(linkIndex=link_id)
        info = DynamicsInfo(*ret)

        if info.contact_damping == -1:
            info.contact_damping = None
        if info.contact_stiffness == -1:
            info.contact_stiffness = None

        return info

    def set_dynamics(self, name: str = None, **kwargs):
        if name is None:
            link_id = -1
        else:
            link_id = self.links[name]

        def convert(s):
            return re.sub('_(.)', lambda p: p[1].upper(), s)

        real_args = {convert(k): v for k, v in kwargs.items()}

        self.client.changeDynamics(linkIndex=link_id, **real_args)

    @staticmethod
    def load_urdf(scene: Scene, path: str, flags=pybullet.URDF_USE_SELF_COLLISION):
        body_id = scene._conn.client.loadURDF(
            path, [0, 0, 0], flags=flags)
        return Robot(body_id, scene)
