import json
import logging
import threading
import time
from collections.abc import Callable

from websockets.sync.client import connect

logger = logging.getLogger(__name__)


class OpenArmBridgeClient:
    """线程安全的 robot_bridge WebSocket 客户端。"""

    def __init__(self, url: str):
        self.url = url
        self._ws = None
        self._send_lock = threading.Lock()
        self._state_ready = threading.Condition()
        self._state: dict = {}
        self._seq = 0
        self._running = False
        self._rx_thread = None
        self._control_thread = None
        self._target: tuple[list[float], list[float]] | None = None
        self._target_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._running and self._ws is not None

    def connect(self) -> None:
        if self.is_connected:
            return
        self._ws = connect(self.url)
        self._running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()
        logger.info("OpenArm bridge 已连接: %s", self.url)

    def close(self) -> None:
        self._running = False
        with self._state_ready:
            self._state_ready.notify_all()
        if self._ws is not None:
            self._ws.close()
            self._ws = None
        logger.info("OpenArm bridge 已断开")

    def get_state(self) -> dict:
        with self._state_ready:
            return dict(self._state)

    def wait_for_state(
        self,
        predicate: Callable[[dict], bool],
        timeout: float,
        description: str,
    ) -> dict:
        deadline = time.monotonic() + timeout
        with self._state_ready:
            while self._running:
                if predicate(self._state):
                    return dict(self._state)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._state_ready.wait(min(remaining, 0.1))
        raise TimeoutError(f"等待 {description} 超时（{timeout:.1f}s）")

    def start_control(self, rate_hz: int) -> None:
        if self._control_thread is not None and self._control_thread.is_alive():
            return
        self._control_thread = threading.Thread(
            target=self._control_loop, args=(rate_hz,), daemon=True
        )
        self._control_thread.start()
        logger.info("策略控制发送循环已启动: %dHz", rate_hz)

    def set_arm_target(self, left: list[float], right: list[float]) -> None:
        with self._target_lock:
            self._target = (list(left), list(right))

    def _control_loop(self, rate_hz: int) -> None:
        period = 1.0 / rate_hz
        while self._running:
            started = time.monotonic()
            with self._target_lock:
                target = self._target
            if target is not None:
                left, right = target
                self._send_command(
                    "arm_pd",
                    {
                        "left": left[:7],
                        "right": right[:7],
                        "left_gripper": left[7],
                        "right_gripper": right[7],
                    },
                )
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _receive_loop(self) -> None:
        try:
            for raw in self._ws:
                frame = json.loads(raw)
                if frame.get("type") != "state":
                    continue
                with self._state_ready:
                    self._state = frame.get("data", {})
                    self._state_ready.notify_all()
        except Exception as exc:
            if self._running:
                logger.error("OpenArm bridge 接收中断: %s", exc)
        finally:
            self._running = False
            with self._state_ready:
                self._state_ready.notify_all()

    def _send_command(self, target: str, data: dict) -> None:
        self._seq += 1
        frame = {
            "type": "command",
            "seq": self._seq,
            "time": time.time(),
            "target": target,
            "data": data,
        }
        with self._send_lock:
            self._ws.send(json.dumps(frame, separators=(",", ":")))
