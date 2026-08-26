from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("openarm_bridge_teleop")
@dataclass(kw_only=True)
class OpenArmBridgeTeleopConfig(TeleoperatorConfig):
    id: str | None = "openarm_bridge_teleop"
    ws_url: str = "ws://127.0.0.1:9000"
    state_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        if self.state_timeout_s <= 0:
            raise ValueError("state_timeout_s 必须大于 0")
