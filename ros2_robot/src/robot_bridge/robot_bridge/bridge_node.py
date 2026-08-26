"""桥接节点：组装 protocol / command_cache / ros_interface / ws_server。

线程模型：
  - WebSocket 线程：只解析帧，写命令缓存或 pending 队列，绝不直接碰 ROS。
  - ROS 线程（定时器）：唯一执行 ROS 收发的地方。
      * control_rate (默认 100Hz): arm_pd 指令 ZOH 重复发布
      * 50Hz : 执行 pending 一次性指令 + 广播状态
"""

import collections
import os
import signal
import threading
import time

import rclpy
from rclpy.node import Node

from . import protocol
from .command_cache import CommandCache
from .config_loader import load_config
from .process_manager import ProcessManager
from .ros_interface import RosInterface, LEFT, RIGHT


class BridgeNode(Node):
    def __init__(self):
        super().__init__("robot_bridge")

        # 先加载集中配置：接口名 / launch / 限位 都在这里，作为各参数默认值来源
        self.declare_parameter("config_file", "")
        self.config = load_config(self.get_parameter("config_file").value)
        srv = self.config["server"]

        # 服务器参数：config 提供默认值，launch/命令行参数可覆盖
        self.declare_parameter("host", srv["host"])
        self.declare_parameter("port", int(srv["port"]))
        self.declare_parameter("control_rate", float(srv["control_rate"]))
        self.declare_parameter("state_rate", float(srv["state_rate"]))
        self.declare_parameter("command_ttl_ms", float(srv["command_ttl_ms"]))
        self.declare_parameter("arm_mode", srv["default_arm_mode"])

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        control_rate = float(self.get_parameter("control_rate").value)
        state_rate = float(self.get_parameter("state_rate").value)
        self._ttl = float(self.get_parameter("command_ttl_ms").value)
        self._chassis_ttl_ms = float(
            self.config["chassis"]["command_timeout_sec"]) * 1000.0
        arm_mode = str(self.get_parameter("arm_mode").value)
        if arm_mode not in protocol.ARM_MODES:
            raise ValueError(f"不支持的 arm_mode 参数: {arm_mode}")

        self.ros = RosInterface(self, self.config)
        self.ros.on_move_pose_result = self._on_move_pose_result
        self.cache = CommandCache(default_ttl_ms=self._ttl)
        self.processes = ProcessManager(self.config, logger=self.get_logger())

        self.arm_mode = arm_mode
        self._validate = self._build_validator()
        self._pending = collections.deque()
        self._pending_lock = threading.Lock()
        self._state_seq = 0
        self._log_times = {}
        self._startup_wait = None
        self._shutting_down = False

        # WebSocket 延迟导入，避免无依赖时 import 失败影响其它模块
        from .ws_server import WsServer
        self.ws = WsServer(host, port, self._on_message, logger=self.get_logger())
        self.ws.start()

        self._control_timer = self.create_timer(1.0 / control_rate, self._control_loop)
        self._aux_timer = self.create_timer(1.0 / state_rate, self._aux_loop)
        self.get_logger().info(
            f"robot_bridge 就绪：default_arm_mode={self.arm_mode}, "
            f"control {control_rate:.0f}Hz / state {state_rate:.0f}Hz")

    def _build_validator(self):
        """按 config 决定是否启用 schema 校验；关闭或不可用时返回 None。"""
        if not self.config["server"].get("validate_incoming"):
            return None
        from ament_index_python.packages import get_package_share_directory
        from .schema import make_validator
        schema_path = os.path.join(
            get_package_share_directory("robot_bridge"),
            "config", "schemas", "frames.schema.json")
        validator = make_validator(schema_path)
        if validator is None:
            self.get_logger().warning("validate_incoming=true 但 jsonschema 不可用，跳过校验")
        else:
            self.get_logger().info(f"入站帧校验已启用: {schema_path}")
        return validator

    # ==================== WebSocket 线程：仅入队 ====================
    def _on_message(self, frame: dict, client_id: int):
        if self._shutting_down:
            return protocol.make_error("robot_bridge 正在关闭", code="SHUTTING_DOWN", req=frame)
        if self._validate is not None:
            err = self._validate(frame)
            if err is not None:
                return protocol.make_error(f"帧不符合 schema: {err}", code="BAD_FRAME", req=frame)
        ftype = frame.get("type")
        if ftype == protocol.TYPE_COMMAND:
            self._ingest_command(frame)
            return None
        if ftype == protocol.TYPE_REQUEST:
            self._push({"kind": "request", "frame": frame})
            return protocol.make_response(frame, ok=True, message="accepted")
        return protocol.make_error(f"不支持的帧类型: {ftype}", code="BAD_TYPE", req=frame)

    def _ingest_command(self, frame: dict):
        target = frame.get("target")
        data = frame.get("data", {})
        ttl = frame.get("ttl_ms", self._ttl)
        # chassis 50Hz 持续发送，不打 rx 日志以免刷屏
        if target != "chassis":
            self._log_every(
                f"rx:{target}",
                1.0,
                f"WS rx command target={target} keys={list(data.keys())} ttl_ms={ttl}",
            )

        if target == "arm_pd":
            self.cache.set("arm_pd", data, ttl_ms=ttl)
        elif target == "chassis":
            self.cache.set("chassis", data, ttl_ms=self._chassis_ttl_ms)
        elif target == "chassis_state":
            self._push({"kind": "chassis_state", "data": data})
        elif target == "arm_traj":
            self._push({"kind": "arm_traj", "data": data})
        elif target == "arm_cartesian":
            self._push({"kind": "arm_cartesian", "data": data})
        elif target == "gripper":
            self._push({"kind": "gripper", "data": data})
        elif target == "lift":
            self._push({"kind": "lift", "data": data})
        elif target == "waist":
            self._push({"kind": "waist", "data": data})
        # 未知 target 静默忽略，避免高频日志刷屏

    def _push(self, item):
        with self._pending_lock:
            self._pending.append(item)

    def _drain(self):
        with self._pending_lock:
            items = list(self._pending)
            self._pending.clear()
        return items

    # ==================== ROS 线程：control_rate 重发 ====================
    def _control_loop(self):
        if self._shutting_down:
            return
        if self.arm_mode != protocol.ARM_GRAVITY_PD:
            return
        data, expired = self.cache.get("arm_pd")
        if data is None or expired:
            return  # 超时不再重复，控制器保持上一指令，避免发散
        left = data.get("left")
        right = data.get("right")
        if left is not None:
            self.ros.publish_arm_pd(LEFT, left, data.get("left_gripper"))
        if right is not None:
            self.ros.publish_arm_pd(RIGHT, right, data.get("right_gripper"))
        self._log_every(
            "ros_pub:arm_pd",
            1.0,
            "ROS pub arm_pd "
            f"left={'yes' if left is not None else 'no'} "
            f"right={'yes' if right is not None else 'no'}",
        )

    # ==================== ROS 线程：50Hz 杂务 + 状态 ====================
    def _aux_loop(self):
        if self._shutting_down:
            return
        for item in self._drain():
            self._execute(item)
        self._publish_chassis()
        self._check_startup_ready()
        self._broadcast_state()

    def _publish_chassis(self):
        data, expired = self.cache.get("chassis")
        if data is None:
            return
        if expired:
            self.ros.publish_chassis()
            return
        self.ros.publish_chassis(
            data.get("vx", 0.0),
            data.get("vy", 0.0),
            data.get("wz", 0.0),
        )

    def _execute(self, item):
        kind = item["kind"]
        if kind == "request":
            self._handle_request(item["frame"])
            return
        data = item.get("data", {})
        if kind == "arm_traj":
            self.get_logger().info(
                f"ROS action arm_traj keys={list(data.keys())} duration={data.get('duration', 2.0)}")
            dur = data.get("duration", 2.0)
            if "left" in data:
                self.ros.send_arm_trajectory(LEFT, data["left"], dur, data.get("left_velocity"))
            if "right" in data:
                self.ros.send_arm_trajectory(RIGHT, data["right"], dur, data.get("right_velocity"))
        elif kind == "chassis_state":
            command = data.get("command", "")
            if self.ros.set_chassis_state(command):
                self.get_logger().info(f"ROS pub chassis state command={command}")
        elif kind == "gripper":
            self.get_logger().info(f"ROS send gripper {data}")
            if "left" in data:
                self.ros.send_gripper(LEFT, data["left"])
            if "right" in data:
                self.ros.send_gripper(RIGHT, data["right"])
        elif kind == "lift":
            self.get_logger().info(f"ROS pub lift {data}")
            if "position" in data:
                self.ros.lift_move_absolute(data["position"])
            if "velocity" in data:
                self.ros.lift_set_velocity(data["velocity"])
        elif kind == "waist":
            self.get_logger().info(f"ROS pub waist {data}")
            if "torque" in data or "velocity" in data:
                self.ros.waist_command(
                    data.get("position", 0.0), data.get("velocity", 0.0), data.get("torque", 0.0))
            elif "position" in data:
                self.ros.waist_position(data["position"])
        elif kind == "arm_cartesian":
            self.get_logger().info(f"ROS action /move_pose {data}")
            arm = data.get("arm")
            ok = self.ros.send_arm_pose(
                arm,
                data.get("position", {}),
                data.get("orientation", {}),
                vel_scale=data.get("vel_scale", 0.0),
                acc_scale=data.get("acc_scale", 0.0),
                plan_only=bool(data.get("plan_only", False)),
                frame_id=data.get("frame_id", ""),
            )
            if not ok:
                self.ws.broadcast(protocol.make_event(
                    "move_pose_result",
                    {"arm": arm, "success": False, "error_code": -1,
                     "message": "/move_pose action 不可用或参数非法"}))

    def _on_move_pose_result(self, arm, success, error_code, message):
        self.ws.broadcast(protocol.make_event(
            "move_pose_result",
            {"arm": arm, "success": success,
             "error_code": error_code, "message": message}))

    def _handle_request(self, frame: dict):
        action = frame.get("action")
        if action == "startup":
            self.get_logger().info(f"WS rx request startup data={frame.get('data', {})}")
            self._do_startup(frame)
        elif action == "emergency_stop":
            self.get_logger().warning("WS rx request emergency_stop")
            ok = self.ros.emergency_stop()
            self.ws.broadcast(protocol.make_event(
                "emergency_stop", {"ok": ok}, req_id=frame.get("id")))
        elif action == "reset":
            self.get_logger().warning("WS rx request reset")
            self._do_reset(frame)
        else:
            self.ws.broadcast(protocol.make_error(
                f"未知 action: {action}", code="BAD_ACTION", req=frame))

    def _do_startup(self, frame: dict):
        data = frame.get("data", {})
        mode = data.get("arm_mode", protocol.ARM_TRAJECTORY)
        if mode not in protocol.ARM_MODES:
            self.ws.broadcast(protocol.make_error(
                f"非法 arm_mode: {mode}", code="BAD_ARG", req=frame))
            return
        self.arm_mode = mode
        self.cache.clear("arm_pd")
        components = data.get("components")
        if components is None:
            components = self.config["startup"]["default_components"]
        show_terminal = bool(data.get("show_terminal", False))
        launch_result = self.processes.startup(mode, components, show_terminal=show_terminal)
        timeout = float(data.get("ready_timeout_sec", self.config["startup"]["ready_timeout_sec"]))
        self._startup_wait = {
            "id": frame.get("id"),
            "arm_mode": mode,
            "components": components,
            "show_terminal": show_terminal,
            "launch": launch_result,
            "deadline": time.monotonic() + timeout,
        }
        self.get_logger().info(
            f"startup 已发起，等待 ROS 接口 ready，arm_mode={mode}, "
            f"components={components}, show_terminal={show_terminal}, timeout={timeout:.1f}s")

    def _check_startup_ready(self):
        if self._startup_wait is None:
            return
        item = self._startup_wait
        readiness = self.ros.readiness(item["arm_mode"], item["components"])
        self._log_every("startup_readiness", 1.0, f"startup readiness: {readiness}")

        if readiness["ready"]:
            if "lift" in item["components"]:
                self.ros.lift_enable()
            if "waist" in item["components"]:
                self.ros.waist_enable(True)
            self.get_logger().info(
                f"startup 完成，ROS 接口已 ready，arm_mode={item['arm_mode']}")
            self.ws.broadcast(protocol.make_event(
                "startup_completed",
                {
                    "arm_mode": item["arm_mode"],
                    "launch": item["launch"],
                    "readiness": readiness,
                },
                req_id=item["id"]))
            self._startup_wait = None
            return

        if time.monotonic() >= item["deadline"]:
            self.get_logger().error(f"startup 超时，ROS 接口未 ready: {readiness}")
            self.ws.broadcast(protocol.make_event(
                "startup_failed",
                {
                    "arm_mode": item["arm_mode"],
                    "launch": item["launch"],
                    "readiness": readiness,
                },
                req_id=item["id"]))
            self._startup_wait = None

    def _do_reset(self, frame: dict):
        with self._pending_lock:
            self._pending.clear()
        self.cache.clear()
        self._startup_wait = None
        self.arm_mode = protocol.ARM_TRAJECTORY
        self.ros.publish_chassis()
        self.ros.reset_state()
        shutdown_result = self.processes.shutdown()
        self.get_logger().warning(f"reset 完成，已停止启动进程: {shutdown_result}")
        self.ws.broadcast(protocol.make_event(
            "reset_completed",
            {
                "arm_mode": self.arm_mode,
                "shutdown": shutdown_result,
            },
            req_id=frame.get("id")))

    # ==================== 状态广播 ====================
    def _broadcast_state(self):
        if self._shutting_down:
            return
        self._state_seq += 1
        snap = self.ros.snapshot()
        snap["arm_mode"] = self.arm_mode
        self.ws.broadcast(protocol.make_state(snap, self._state_seq))
        # 默认不打周期状态日志；需要时设环境变量 ROBOT_BRIDGE_LOG_STATE=1
        if os.environ.get("ROBOT_BRIDGE_LOG_STATE", "").strip() in ("1", "true", "TRUE"):
            self._log_every(
                "state_tx",
                2.0,
                "WS tx state "
                f"seq={self._state_seq} "
                f"left_pos={len(snap['left_arm'].get('position', []))} "
                f"right_pos={len(snap['right_arm'].get('position', []))} "
                f"teleop_valid={snap['teleop_action']['valid']} "
                f"chassis_keys={list(snap['chassis'].keys())} "
                f"lift_keys={list(snap['lift'].keys())} "
                f"waist_keys={list(snap['waist'].keys())}",
            )

    def _log_every(self, key: str, interval: float, msg: str):
        now = time.monotonic()
        last = self._log_times.get(key, 0.0)
        if now - last >= interval:
            self._log_times[key] = now
            self.get_logger().info(msg)

    def destroy_node(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        # Ctrl+C / 关终端：先急停再杀子进程，否则 emergency_stop_node 会被一起拆掉
        try:
            self.cache.clear("chassis")
            self.ros.publish_chassis()
            ok = self.ros.emergency_stop(wait_sec=2.0)
            msg = f"[robot_bridge] shutdown 触发急停 ok={ok}"
            try:
                self.get_logger().warning(msg)
            except Exception:  # noqa: BLE001
                print(msg, flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[robot_bridge] shutdown emergency_stop error: {e}", flush=True)
        with self._pending_lock:
            self._pending.clear()
        try:
            self.destroy_timer(self._control_timer)
            self.destroy_timer(self._aux_timer)
        except Exception:
            pass
        try:
            self.ws.stop()
        except Exception:
            pass
        try:
            shutdown_result = self.processes.shutdown()
            print(
                f"[robot_bridge] shutdown 已停止启动进程: {shutdown_result}",
                flush=True,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[robot_bridge] shutdown process error: {e}", flush=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0

    def _request_shutdown(signum, _frame):
        # SIGHUP(关终端) / SIGTERM：走与 Ctrl+C 相同的 graceful 路径，确保 destroy_node 里急停
        if rclpy.ok():
            rclpy.try_shutdown()

    for sig in (signal.SIGHUP, signal.SIGTERM):
        try:
            signal.signal(sig, _request_shutdown)
        except (ValueError, OSError):
            pass

    try:
        node = BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001 让 launch 日志显示清楚的启动失败原因
        exit_code = 1
        if node is not None and rclpy.ok():
            node.get_logger().error(str(e))
        else:
            print(f"robot_bridge 启动失败: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    main()
