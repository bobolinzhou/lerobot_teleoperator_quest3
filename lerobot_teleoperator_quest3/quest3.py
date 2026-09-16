import logging
import time
from typing import Any
import threading
import math

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceNotConnectedError
from .config_quest3 import Quest3Config
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

logger = logging.getLogger(__name__)


class Quest3(Teleoperator):
    config_class = Quest3Config
    name = "quest3"

    def __init__(self, config: Quest3Config):
        super().__init__(config)
        self.config = config

        self._connected = False
        self.node = None

        self._lock = threading.Lock()

        self._latest_action: dict[str, float] | None = None
        self._latest_action_timestamp: float | None = None
        self._executor = None
        self._executor_thread = None

        
    def connect(self, calibrate: bool = True) -> None:
        #订阅相关话题，打包关节数据
        #话题发布类型为sensor_msgs/msg/JointState
        if self._connected:
            logger.warning(f"{self} is already connected.")
            return

        #初始化ros
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("quest3_teleoperator_node")
        self.node.create_subscription(
            JointState,  # 话题消息类型
            self.config.topic,
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        self._connected = True
        if calibrate:
            self.calibrate()

    #订阅回调
    def joint_state_callback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            logger.warning(f"Received joint state message with unexpected number of positions: {len(msg.position)}")
            return

        positions = dict(zip(msg.name, msg.position))
        missing_joints = [joint for joint in self.config.ros_joint_names if joint not in positions]

        if missing_joints:
            logger.warning(f"Missing joint positions for joints: {missing_joints}")
            return
        action = {}

        for ros_name, action_key in zip(
            self.config.ros_joint_names,
            self.config.action_keys,
            strict=True,
        ):
            value = positions[ros_name]
            if ros_name == "gripper":
                value = value * 1000.0  # m -> mm
            else:
                value = math.degrees(value)  # rad -> degree

            action[action_key] = value

        #action = {
        #    action_key: positions[ros_name] for ros_name, action_key in zip(
        #        self.config.ros_joint_names, self.config.action_keys)
        #}

        with self._lock:
            self._latest_action = action
            self._latest_action_timestamp = time.monotonic()

       
    def disconnect(self):
        #停止订阅相关话题
        if not self._connected:
            return

        if self._executor is not None:
            self._executor.shutdown()

        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)

        if self.node is not None:
            self.node.destroy_node()

        self.node = None
        self._executor = None
        self._executor_thread = None
        self._connected = False

    #相关配置和标定由遥操节点在发布前就已经完成
    def configure(self) -> None:
        pass

    def calibrate(self):
        pass

    def get_action(self) -> dict[str, float]:
        #获取关节数据，返回字典
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with self._lock:
            if self._latest_action is None or self._latest_action_timestamp is None:
                raise RuntimeError(
                    f"No action received yet from {self.config.topic}."
                )

            action = self._latest_action.copy()
            age_s = time.monotonic() - self._latest_action_timestamp


        if age_s > self.config.stale_action_timeout:
            raise RuntimeError(f"Action from {self.config.topic} is stale (age: {age_s:.3f}s).")

        return action


    def send_feedback(self, feedback: dict[str, Any]) -> None:
        #发送反馈数据到quest
        pass

    #描述action特征的函数，返回字典
    @property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in self.config.action_keys}

    #描述机器人预期反馈动作的结构和类型
    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        #检查是否连接成功
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        #检查是否校准成功
        return True

    

    
