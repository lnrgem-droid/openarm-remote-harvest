import logging
import time
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_not_connected

from .config_openarm_bridge import OpenArmBridgeConfig
from .joint_schema import (
    ALL_JOINTS,
    GRIPPER_OPEN_M,
    action_to_values,
    values_to_action,
)
from .ws_client import OpenArmBridgeClient

logger = logging.getLogger(__name__)


class OpenArmBridge(Robot):
    config_class = OpenArmBridgeConfig
    name = "openarm_bridge"

    def __init__(self, config: OpenArmBridgeConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.client = OpenArmBridgeClient(config.ws_url)
        self._last_status_log = 0.0

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {f"{name}.pos": float for name in ALL_JOINTS}
        features.update(
            {
                name: (camera.height, camera.width, 3)
                for name, camera in self.cameras.items()
            }
        )
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in ALL_JOINTS}

    @property
    def is_connected(self) -> bool:
        return self.client.is_connected and all(camera.is_connected for camera in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self.is_connected:
            return
        try:
            self.client.connect()
            self.client.wait_for_state(
                self._has_ready_state,
                self.config.state_timeout_s,
                "gravity_pd 模式和双臂 /joint_states",
            )
            for camera in self.cameras.values():
                camera.connect()
            if self.config.control_authority == "policy":
                self.client.start_control(self.config.policy_command_rate_hz)
        except Exception:
            self.disconnect()
            raise
        logger.info(
            "OpenArm LeRobot 就绪: authority=%s cameras=%s",
            self.config.control_authority,
            list(self.cameras),
        )

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        predicate = (
            self._has_record_state
            if self.config.control_authority == "external"
            else self._has_ready_state
        )
        state = self.client.wait_for_state(
            predicate,
            self.config.state_timeout_s,
            "有效 ROS2 状态",
        )
        left = list(state["left_arm"]["position"][:8])
        right = list(state["right_arm"]["position"][:8])
        left[7] = self._normalize_gripper(left[7])
        right[7] = self._normalize_gripper(right[7])
        observation: RobotObservation = values_to_action(left, right)
        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()

        now = time.monotonic()
        if now - self._last_status_log >= 2.0:
            self._last_status_log = now
            logger.info(
                "采集状态: joints=16 teleop_valid=%s cameras=%d",
                state.get("teleop_action", {}).get("valid", False),
                len(self.cameras),
            )
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if self.config.control_authority == "policy":
            left, right = action_to_values(action)
            self.client.set_arm_target(left, right)
        return action

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        self.client.close()

    @staticmethod
    def _normalize_gripper(position_m: float) -> float:
        return max(0.0, min(1.0, float(position_m) / GRIPPER_OPEN_M))

    @staticmethod
    def _has_joint_state(state: dict) -> bool:
        return all(
            len(state.get(f"{side}_arm", {}).get("position", [])) >= 8
            for side in ("left", "right")
        )

    @classmethod
    def _has_ready_state(cls, state: dict) -> bool:
        return state.get("arm_mode") == "gravity_pd" and cls._has_joint_state(state)

    @classmethod
    def _has_record_state(cls, state: dict) -> bool:
        return cls._has_ready_state(state) and state.get("teleop_action", {}).get("valid") is True
