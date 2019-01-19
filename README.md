# silverbullet

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/c9b3d97530be4717bd5dc3c4409bc545)](https://app.codacy.com/app/coord-e/silverbullet?utm_source=github.com&utm_medium=referral&utm_content=MonoMotion/silverbullet&utm_campaign=Badge_Grade_Dashboard)
[![Travis CI](https://img.shields.io/travis/MonoMotion/silverbullet.svg?style=flat-square)](https://travis-ci.org/MonoMotion/silverbullet)
[![pypi](https://img.shields.io/pypi/v/silverbullet.svg)](https://pypi.org/project/silverbullet/)
[![Reviewed by Hound](https://img.shields.io/badge/Reviewed_by-Hound-8E64B0.svg?style=flat-square)](https://houndci.com)
[![license](https://img.shields.io/github/license/MonoMotion/silverbullet.svg?style=flat-square)](LICENSE)
[![GitHub release](https://img.shields.io/github/release/MonoMotion/silverbullet.svg?style=flat-square)](https://github.com/MonoMotion/silverbullet/releases)

shallow but powerful wrapper around pybullet

## Installation

```shell
pip install silverbullet
```

## Usage

```python
from silverbullet import Scene, Robot

# GUI mode
# from silverbullet import Connection, Mode
# conn = Connection(mode=Mode.GUI)
# scene = Scene(timestep=0.01, frame_skip=4, connection=conn)

scene = Scene(timestep=0.01, frame_skip=4)
robot = Robot.load_urdf(scene, "robot.urdf")
robot.bring_on_the_ground()

while True:

  # You can control and observe the robot
  # e.g. robot.set_joint_torque('joint1', 1)

  do_something(robot)

  # This will step simulation
  scene.step()
```
