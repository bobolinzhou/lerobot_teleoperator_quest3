# LeRobot Quest 3 Teleoperator

`lerobot_teleoperator_quest3` is a third-party [LeRobot](https://github.com/huggingface/lerobot) teleoperator plugin that receives joint targets from ROS 2 and exposes them as LeRobot actions for a Piper robot adapter. It was developed and tested with a Meta Quest 3 teleoperation pipeline, but the plugin itself is not tied to Quest 3 hardware.

The plugin does not connect to the Quest headset directly. A separate ROS 2 teleoperation/IK pipeline must publish the target joint state consumed by this package.

## Publisher-independent input

The source of the ROS 2 data can be any device or program. For example, `/control/joint_states` may be published by a Quest 3 pipeline, another VR headset, a joystick or keyboard controller, MoveIt, a simulator, a motion-capture system, another robot, or a custom planning node.

The publisher implementation and hardware do not matter to this plugin. Only the ROS 2 interface contract matters: messages must use `sensor_msgs/msg/JointState`, contain the configured joint names, and use the units documented below. In other words, the **publisher may be arbitrary, but the message type and data format are not arbitrary**.

## Data flow

```text
Any control source
(Quest 3, other VR, simulator, planner, etc.)
        ↓
ROS 2 publisher / optional IK nodes
        ↓  /control/joint_states
lerobot_teleoperator_quest3
        ↓  LeRobot action dictionary
lerobot_robot_piper
        ↓
Piper SDK
```

## Compatibility

- Python 3.10
- ROS 2 Humble
- LeRobot `>=0.4.4,<0.5.0`
- Input message: `sensor_msgs/msg/JointState`

## ROS 2 input

The default input topic is:

```text
/control/joint_states
```

Expected message type:

```text
sensor_msgs/msg/JointState
```

Expected joint names and order:

```yaml
name:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - gripper
```

Expected `position` units:

| Input name | Meaning | Input unit |
|---|---|---|
| `joint1` ... `joint6` | Revolute joint target | radians |
| `gripper` | Gripper opening | metres |

`velocity` and `effort` may be empty because this plugin only reads `name` and `position`.

Example message:

```yaml
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: [joint1, joint2, joint3, joint4, joint5, joint6, gripper]
position: [0.5, 1.0, -0.2, 0.1, -0.8, 0.3, 0.07]
velocity: []
effort: []
```

## LeRobot action output

The plugin maps the ROS names to the action keys required by the Piper adapter:

| ROS input | LeRobot output | Conversion |
|---|---|---|
| `joint1` | `Joint_1.pos` | radians to degrees |
| `joint2` | `Joint_2.pos` | radians to degrees |
| `joint3` | `Joint_3.pos` | radians to degrees |
| `joint4` | `Joint_4.pos` | radians to degrees |
| `joint5` | `Joint_5.pos` | radians to degrees |
| `joint6` | `Joint_6.pos` | radians to degrees |
| `gripper` | `Gripper.pos` | metres to millimetres |

For example:

```text
1.0 rad  → 57.2958 degrees
0.07 m   → 70.0 mm
```

The Piper adapter then converts degrees to the Piper SDK joint unit (`0.001°`) and millimetres to the SDK gripper unit (`0.001 mm`).

## Installation

Activate the same Python environment that contains LeRobot, then install this package in editable mode:

```bash
conda activate data_collect
source /opt/ros/humble/setup.bash

git clone git@github.com:bobolinzhou/lerobot_teleoperator_quest3.git
cd lerobot_teleoperator_quest3
python3 -m pip install --no-deps -e .
```

Verify the installation:

```bash
python3 -m pip show lerobot_teleoperator_quest3

python3 -c "from lerobot_teleoperator_quest3 import Quest3, Quest3Config; c = Quest3Config(); print(c.topic, c.ros_joint_names, Quest3.__abstractmethods__)"
```

The abstract-method set should be empty.

## Checking the ROS 2 input

Before starting LeRobot, confirm that the topic exists and is publishing:

```bash
ros2 topic info -v /control/joint_states
ros2 topic echo --once /control/joint_states
ros2 topic hz /control/joint_states
```

The default stale-action timeout is `0.2` seconds. If the ROS publisher stops or becomes too slow, `get_action()` raises an error instead of returning an old target.

## LeRobot registration

The distribution name starts with `lerobot_teleoperator_`, so LeRobot discovers it as a third-party plugin. The registered teleoperator type is:

```bash
--teleop.type=quest3
```

Robot, camera, and dataset arguments depend on the rest of the LeRobot setup.

## Development workflow

Because the package is installed with `pip install -e .`, edits to `quest3.py` and `config_quest3.py` are used by the next Python process automatically.

After changing Python files:

1. Save the files.
2. Stop the running LeRobot process.
3. Start the LeRobot command again.

Re-run `pip install --no-deps -e .` only after changing package metadata, dependencies, or the directory layout in `pyproject.toml`.

## Safety

Verify joint names, units, joint limits, gripper range, and emergency-stop behavior before enabling a physical robot. Test the ROS-to-LeRobot conversion without sending actions to the arm first.
