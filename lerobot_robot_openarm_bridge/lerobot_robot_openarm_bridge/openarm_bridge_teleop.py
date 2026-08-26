import logging
from functools import cached_property
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.lerobot_types import RobotAction
from lerobot.utils.decorators import check_if_not_connected

from .config_openarm_bridge_teleop import OpenArmBridgeTeleopConfig
from .joint_schema import ALL_JOINTS, values_to_action
from .ws_client import OpenArmBridgeClient

logger = logging.getLogger(__name__)


class OpenArmBridgeTeleop(Teleoperator):
    config_class = OpenArmBridgeTeleopConfig
    name = "openarm_bridge_teleop"

    def __init__(self, config: OpenArmBridgeTeleopConfig):
        super().__init__(config)
        self.config = config
        self.client = OpenArmBridgeClient(config.ws_url)

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in ALL_JOINTS}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.client.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self.client.connect()
        self.client.wait_for_state(bool, self.config.state_timeout_s, "robot_bridge state")
        logger.info("OpenArm 被动采集动作源已连接")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        state = self.client.wait_for_state(
            lambda value: value.get("teleop_action", {}).get("valid") is True,
            self.config.state_timeout_s,
            "有效 VR/IK action",
        )
        action = state["teleop_action"]
        return values_to_action(action["left"], action["right"])

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    def disconnect(self) -> None:
        self.client.close()
