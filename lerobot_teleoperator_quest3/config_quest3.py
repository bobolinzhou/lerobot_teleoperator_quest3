from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple,Any, Dict, Protocol


from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("quest3")
@dataclass
class Quest3Config(TeleoperatorConfig):
    # 话题名，quest数据由该话题发布并读取
    topic: str = "/control/joint_states"

    ros_joint_names: list[str] = field(default_factory=lambda: [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper"
    ])

    action_keys: list[str] = field(default_factory=lambda: [
        "Joint_1.pos",
        "Joint_2.pos",
        "Joint_3.pos",
        "Joint_4.pos",
        "Joint_5.pos",
        "Joint_6.pos",
        "Gripper.pos"
    ])

    # Service exposed by pub_delta_pose.py. It stops Cartesian target
    # publishing and re-anchors the next A-start at the current TCP pose.
    stop_teleop_service: str = "/quest3/stop_teleop"
    stop_teleop_service_timeout_s: float = 2.0
