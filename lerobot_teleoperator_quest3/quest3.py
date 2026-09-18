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
from std_srvs.srv import Trigger

logger = logging.getLogger(__name__)


class Quest3(Teleoperator):
    config_class = Quest3Config
    name = "quest3"

    def __init__(self, config: Quest3Config):
        super().__init__(config)
        self.config = config

        self._connected = False
        self.node = None

        # ROS callbacks update the cached target on the executor thread. The
        # recording loop reads that cache without waiting; the condition is
        # used only at episode boundaries while waiting for the first target.
        self._action_condition = threading.Condition()

        self._latest_action: dict[str, float] | None = None
        self._executor = None
        self._executor_thread = None
        self._stop_teleop_client = None

        
    def connect(self, calibrate: bool = True) -> None:
        #订阅相关话题，打包关节数据
        #话题发布类型为sensor_msgs/msg/JointState
        if self._connected:
            logger.warning(f"{self} is already connected.")
            return

        # A reconnect must wait for a message from the new subscription, not
        # reuse an action cached by the previous ROS executor.
        with self._action_condition:
            self._latest_action = None

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
        self._stop_teleop_client = self.node.create_client(
            Trigger,
            self.config.stop_teleop_service,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        self._connected = True
        if calibrate:
            self.calibrate()

        logger.info(
            "Waiting for the first valid action from %s. Press A to start episode 1.",
            self.config.topic,
        )
        self._wait_for_first_action()
        logger.info("Quest action received. Episode 1 is ready to start.")

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


        with self._action_condition:
            self._latest_action = action
            self._action_condition.notify_all()

       
    def disconnect(self):
        #停止订阅相关话题
        if not self._connected:
            return

        # Wake any get_action() call that is waiting for the publisher to
        # resume before shutting down the executor.
        with self._action_condition:
            self._connected = False
            self._action_condition.notify_all()

        if self._executor is not None:
            self._executor.shutdown()

        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)

        if self.node is not None:
            self.node.destroy_node()

        self.node = None
        self._executor = None
        self._executor_thread = None
        self._stop_teleop_client = None

    #相关配置和标定由遥操节点在发布前就已经完成
    def configure(self) -> None:
        pass

    def calibrate(self):
        pass

    def _wait_for_first_action(self) -> dict[str, float]:
        """Wait at an episode boundary until Quest publishes its first target."""
        with self._action_condition:
            while self._latest_action is None:
                if not self._connected:
                    raise DeviceNotConnectedError(f"{self} is not connected.")
                self._action_condition.wait(timeout=0.1)

            return self._latest_action.copy()

    def get_action(self) -> dict[str, float]:
        """Return the most recent Quest target without waiting for a new message."""
        with self._action_condition:
            if not self._connected:
                raise DeviceNotConnectedError(f"{self} is not connected.")
            if self._latest_action is None:
                raise RuntimeError(
                    "No Quest action is available. Wait for the episode-start "
                    "handshake before entering the recording loop."
                )
            return self._latest_action.copy()

    def pause_for_reset(self) -> None:
        """Stop Quest target publishing before the robot starts its reset."""
        if not self._connected or self._stop_teleop_client is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        timeout_s = self.config.stop_teleop_service_timeout_s
        if not self._stop_teleop_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(
                f"Quest stop service {self.config.stop_teleop_service} was not "
                f"available within {timeout_s:.1f}s. Restart the quest-only launch."
            )

        future = self._stop_teleop_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Timed out calling Quest stop service "
                    f"{self.config.stop_teleop_service}."
                )
            time.sleep(0.01)

        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"Failed to pause Quest teleoperation: {message}")

        logger.info("Quest teleoperation stopped for robot reset: %s", response.message)

    def clear_action_after_reset(self) -> None:
        """Discard the previous episode target after the robot reaches reset."""
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with self._action_condition:
            self._latest_action = None
        logger.info("Cleared the previous Quest target after robot reset.")

    def wait_for_episode_start(self, episode_number: int) -> None:
        """Wait for a fresh A-start before starting the episode clock."""
        logger.info(
            "Robot reset is complete. Press A to start episode %d; recording "
            "has not started yet.",
            episode_number,
        )
        self._wait_for_first_action()
        logger.info("Fresh Quest action received. Starting episode %d now.", episode_number)


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

    

    
