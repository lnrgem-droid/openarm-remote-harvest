from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

CONTROL_AUTHORITIES = ("external", "policy")


@RobotConfig.register_subclass("openarm_bridge")
@dataclass(kw_only=True)
class OpenArmBridgeConfig(RobotConfig):
    id: str | None = "openarm_bridge"
    ws_url: str = "ws://127.0.0.1:9000"
    # draccus CLI 不支持 typing.Literal，用 str + 校验
    control_authority: str = "external"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    rgbd_endpoint: str | None = None
    rgbd_roles: tuple[str, ...] = ("left_wrist", "right_wrist", "chest")
    state_timeout_s: float = 5.0
    policy_command_rate_hz: int = 50

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.control_authority not in CONTROL_AUTHORITIES:
            raise ValueError(
                f"control_authority 必须是 {CONTROL_AUTHORITIES} 之一，当前为 {self.control_authority!r}"
            )
        if self.state_timeout_s <= 0:
            raise ValueError("state_timeout_s 必须大于 0")
        if self.policy_command_rate_hz <= 0:
            raise ValueError("policy_command_rate_hz 必须大于 0")
