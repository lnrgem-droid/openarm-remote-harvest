"""ROS2 接口封装。

只负责和 ROS2 话题/服务/动作打交道，对外暴露语义化方法，
不感知 WebSocket / JSON。状态订阅缓存在本地，由 snapshot() 聚合输出。
对照 interface_list.md 实现。
"""

import json
import math
import threading
import time

import rclpy
from std_msgs.msg import Bool, Float64, Float64MultiArray, String, UInt16
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf2_msgs.msg import TFMessage
from rclpy.action import ActionClient
from rclpy.duration import Duration

from openarm_move_pose.action import MovePose

LEFT = "left"
RIGHT = "right"


class RosInterface:
    def __init__(self, node, config: dict):
        self.node = node
        self._lock = threading.Lock()

        # ---- 从 config 读取接口地址与限位 ----
        topics = config["topics"]
        arms = config["arms"]
        depth = topics["qos_depth"]
        self.arm_joints = {LEFT: arms["left_joints"], RIGHT: arms["right_joints"]}
        self.gripper_joints = {
            LEFT: arms["left_gripper_joint"], RIGHT: arms["right_gripper_joint"]}
        self.joint_count = arms["joint_count"]
        self.gripper_open_m = arms["gripper_open_m"]
        self.teleop_action_size = config["teleop"]["action_size"]
        self.teleop_timeout = config["teleop"]["timeout_sec"]
        self.ee_link_to_side = config["ee_frames"]
        self.chassis_limits = config["chassis"]

        # ---- 控制发布器 ----
        self.pub_pos = {
            side: node.create_publisher(
                Float64MultiArray, topics["arm_position_command"][side], depth)
            for side in (LEFT, RIGHT)
        }
        self.pub_cmd_vel = node.create_publisher(Twist, topics["cmd_vel"], depth)
        chassis = topics["chassis"]
        self.pub_chassis_state = node.create_publisher(
            String, chassis["state_command"], depth)
        self.pub_pd = {
            side: node.create_publisher(JointState, topics["arm_pd_command"][side], depth)
            for side in (LEFT, RIGHT)
        }
        self.gripper_action = {
            side: ActionClient(node, GripperCommand, topics["gripper_action"][side])
            for side in (LEFT, RIGHT)
        }
        self.trajectory_action = {
            side: ActionClient(node, FollowJointTrajectory, topics["trajectory_action"][side])
            for side in (LEFT, RIGHT)
        }
        # 笛卡尔位姿：openarm_move_pose 的 /move_pose action
        self.move_pose_action = ActionClient(node, MovePose, topics["move_pose_action"])
        # 由 bridge_node 注入：move_pose 结果事件广播
        self.on_move_pose_result = None

        # 升降
        lift = topics["lift"]
        self.pub_lift_abs = node.create_publisher(Float64, lift["move_absolute"], depth)
        self.pub_lift_vel = node.create_publisher(Float64, lift["set_velocity"], depth)
        self.cli_lift_enable = node.create_client(Trigger, lift["enable"])

        # 腰部
        waist = topics["waist"]
        self.pub_waist_pos = node.create_publisher(Float64, waist["position_command"], depth)
        self.pub_waist_cmd = node.create_publisher(Float64MultiArray, waist["command"], depth)
        self.pub_waist_enable = node.create_publisher(Bool, waist["enable"], depth)

        # 急停
        self.cli_estop = node.create_client(Trigger, topics["emergency_stop_trigger"])

        # ---- 状态订阅 ----
        self._state = {
            "arms": {"name": [], "position": [], "velocity": [], "effort": []},
            "lift": {},
            "waist": {},
            "chassis": {},
            "ee_poses": {},
            "teleop_action": {
                LEFT: {"position": [], "stamp": 0.0, "recv_time": 0.0},
                RIGHT: {"position": [], "stamp": 0.0, "recv_time": 0.0},
            },
        }
        node.create_subscription(JointState, topics["joint_states"], self._on_joint_states, depth)
        node.create_subscription(
            JointState, topics["arm_pd_command"][LEFT], self._on_left_joint_command, depth)
        node.create_subscription(
            JointState, topics["arm_pd_command"][RIGHT], self._on_right_joint_command, depth)
        node.create_subscription(Float64, lift["position"], self._on_lift_pos, depth)
        node.create_subscription(Float64, lift["velocity"], self._on_lift_vel, depth)
        node.create_subscription(Bool, lift["enabled"], self._on_lift_enabled, depth)
        node.create_subscription(Bool, lift["error"], self._on_lift_error, depth)
        node.create_subscription(UInt16, lift["error_code"], self._on_lift_error_code, depth)
        node.create_subscription(String, lift["error_message"], self._on_lift_error_message, depth)
        node.create_subscription(String, lift["status"], self._on_lift_status, depth)
        node.create_subscription(Float64MultiArray, waist["state"], self._on_waist_state, depth)
        node.create_subscription(
            String, chassis["status"], self._on_chassis_status, depth)
        # 末端位姿反馈：openarm_move_pose/ee_pose_publisher → /ee_poses
        node.create_subscription(TFMessage, topics["ee_poses"], self._on_ee_poses, depth)

    # ================= 双臂控制 =================
    def send_arm_trajectory(self, side, positions, duration, velocities=None):
        """轨迹控制：通过 FollowJointTrajectory action 发送一条单点轨迹。"""
        if not self._valid_joint_array(side, "positions", positions):
            return False
        if velocities is not None and not self._valid_joint_array(side, "velocities", velocities):
            return False

        client = self.trajectory_action[side]
        if not client.server_is_ready():
            client.wait_for_server(timeout_sec=0.5)
        if not client.server_is_ready():
            self._log_error(f"{side} FollowJointTrajectory action server 未 ready")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.arm_joints[side]
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        if velocities:
            pt.velocities = [float(x) for x in velocities]
        pt.time_from_start = Duration(seconds=float(duration)).to_msg()
        goal.trajectory.points = [pt]

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, s=side: self._on_trajectory_goal_response(s, f))
        self._log_info(
            f"{side} 轨迹 action goal 已发送: positions={pt.positions}, duration={duration:.3f}s")
        return True

    def _valid_joint_array(self, side, field: str, values) -> bool:
        if len(values) != self.joint_count:
            self._log_error(
                f"{side} 轨迹 {field} 必须是 {self.joint_count} 个关节值，当前为 {len(values)} 个")
            return False
        return True

    def _on_trajectory_goal_response(self, side, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._log_error(f"{side} 轨迹 action goal 被拒绝")
                return
            self._log_info(f"{side} 轨迹 action goal 已接收")
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f, s=side: self._on_trajectory_result(s, f))
        except Exception as e:  # noqa: BLE001
            self._log_error(f"{side} 轨迹 action goal 发送异常: {e}")

    def _on_trajectory_result(self, side, future):
        try:
            result = future.result().result
            self._log_info(
                f"{side} 轨迹 action 完成 error_code={result.error_code} "
                f"error_string={result.error_string!r}")
        except Exception as e:  # noqa: BLE001
            self._log_error(f"{side} 轨迹 action 结果异常: {e}")

    def _log_info(self, msg: str):
        try:
            self.node.get_logger().info(msg)
        except Exception:
            print(f"[robot_bridge][ros] {msg}", flush=True)

    def _log_error(self, msg: str):
        try:
            self.node.get_logger().error(msg)
        except Exception:
            print(f"[robot_bridge][ros][ERROR] {msg}", flush=True)

    def publish_arm_pd(self, side, positions, gripper=None):
        """PD + 重力补偿直驱：JointState.position = 7 关节 (+夹爪 0..1)。"""
        msg = JointState()
        msg.name = list(self.arm_joints[side])
        pos = [float(x) for x in positions]
        if gripper is not None:
            pos = pos + [float(gripper)]
        msg.position = pos
        self.pub_pd[side].publish(msg)

    def publish_arm_position(self, side, positions):
        """位置控制：Float64MultiArray 长度 7（笛卡尔模式 IK 解算后也走这里）。"""
        msg = Float64MultiArray()
        msg.data = [float(x) for x in positions]
        self.pub_pos[side].publish(msg)

    def publish_chassis(self, vx=0.0, vy=0.0, wz=0.0):
        """发布标准底盘速度，并按配置限幅。"""
        msg = Twist()
        msg.linear.x = self._clamp(vx, self.chassis_limits["max_vx"])
        msg.linear.y = self._clamp(vy, self.chassis_limits["max_vy"])
        msg.angular.z = self._clamp(wz, self.chassis_limits["max_wz"])
        self.pub_cmd_vel.publish(msg)

    def set_chassis_state(self, command: str) -> bool:
        """发布底盘状态指令：使能、失能或刹车。"""
        command = str(command).strip().lower()
        if command not in ("enable", "disable", "brake"):
            self._log_error(f"非法底盘状态指令: {command!r}")
            return False
        self.pub_chassis_state.publish(String(data=command))
        return True

    @staticmethod
    def _clamp(value, limit):
        limit = abs(float(limit))
        try:
            value = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    # ================= 笛卡尔位姿 =================
    def send_arm_pose(self, arm, position, orientation,
                      vel_scale=0.0, acc_scale=0.0,
                      plan_only=False, frame_id=""):
        """通过 /move_pose action 发末端位姿目标（openarm_move_pose → MoveIt）。"""
        if arm not in (LEFT, RIGHT):
            self._log_error(f"send_arm_pose 非法 arm={arm!r}")
            return False
        try:
            pos = self._as_point(position)
            quat = self._as_quaternion(orientation)
        except (ValueError, TypeError) as e:
            self._log_error(f"send_arm_pose pose 解析失败: {e}")
            return False

        client = self.move_pose_action
        if not client.server_is_ready():
            client.wait_for_server(timeout_sec=0.5)
        if not client.server_is_ready():
            self._log_error("/move_pose action server 未 ready")
            return False

        goal = MovePose.Goal()
        goal.arm = arm
        goal.pose = Pose(position=pos, orientation=quat)
        goal.frame_id = frame_id or ""
        goal.vel_scale = float(vel_scale)
        goal.acc_scale = float(acc_scale)
        goal.plan_only = bool(plan_only)

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(lambda f, a=arm: self._on_move_pose_goal_response(a, f))
        self._log_info(
            f"/move_pose goal 已发送 arm={arm} pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) "
            f"quat=({quat.x:.3f},{quat.y:.3f},{quat.z:.3f},{quat.w:.3f}) "
            f"vel={vel_scale} acc={acc_scale} plan_only={plan_only}")
        return True

    def _on_move_pose_goal_response(self, arm, future):
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self._emit_move_pose_result(arm, False, -1, f"send_goal 异常: {e}")
            return
        if handle is None or not handle.accepted:
            self._emit_move_pose_result(arm, False, -1, "/move_pose goal 被拒绝")
            return
        handle.get_result_async().add_done_callback(
            lambda f, a=arm: self._on_move_pose_result(a, f))

    def _on_move_pose_result(self, arm, future):
        try:
            wrapped = future.result()
            res = wrapped.result if wrapped is not None else None
            if res is None:
                self._emit_move_pose_result(arm, False, -1, "/move_pose 无 result")
                return
            self._emit_move_pose_result(
                arm, bool(res.success), int(res.error_code), res.message)
        except Exception as e:  # noqa: BLE001
            self._emit_move_pose_result(arm, False, -1, f"get_result 异常: {e}")

    def _emit_move_pose_result(self, arm, success, error_code, message):
        self._log_info(
            f"/move_pose 结果 arm={arm} success={success} code={error_code} msg={message!r}")
        cb = self.on_move_pose_result
        if cb is not None:
            try:
                cb(arm, success, error_code, message)
            except Exception:  # noqa: BLE001 桥接回调异常不应影响 ROS 回调
                pass

    @staticmethod
    def _as_point(value) -> Point:
        if not isinstance(value, dict):
            raise ValueError("position 必须是 {x,y,z} dict")
        return Point(
            x=float(value.get("x", 0.0)),
            y=float(value.get("y", 0.0)),
            z=float(value.get("z", 0.0)),
        )

    @staticmethod
    def _as_quaternion(value) -> Quaternion:
        if not isinstance(value, dict):
            raise ValueError("orientation 必须是 {x,y,z,w} dict")
        return Quaternion(
            x=float(value.get("x", 0.0)),
            y=float(value.get("y", 0.0)),
            z=float(value.get("z", 0.0)),
            w=float(value.get("w", 0.0)),
        )

    # ================= 夹爪 =================
    def send_gripper(self, side, position_norm):
        """ros2_control 模式夹爪：归一化 0..1 -> 0..0.044 米，走动作。"""
        client = self.gripper_action[side]
        if not client.server_is_ready():
            client.wait_for_server(timeout_sec=0.1)
        goal = GripperCommand.Goal()
        goal.command.position = max(0.0, min(1.0, float(position_norm))) * self.gripper_open_m
        goal.command.max_effort = 0.0
        client.send_goal_async(goal)

    # ================= 升降 =================
    def lift_move_absolute(self, position):
        self.pub_lift_abs.publish(Float64(data=float(position)))

    def lift_set_velocity(self, velocity):
        self.pub_lift_vel.publish(Float64(data=float(velocity)))

    def lift_enable(self):
        if self.cli_lift_enable.service_is_ready():
            self.cli_lift_enable.call_async(Trigger.Request())
            return True
        return False

    def readiness(self, arm_mode: str, components: list[str]) -> dict:
        """返回启动后各组件 ROS 接口是否已可用。"""
        status = {}
        if "arms" in components:
            if arm_mode == "gravity_pd":
                status["arms"] = {
                    "ready": (
                        self.pub_pd[LEFT].get_subscription_count() > 0
                        and self.pub_pd[RIGHT].get_subscription_count() > 0
                    ),
                    "left_joint_command_subs": self.pub_pd[LEFT].get_subscription_count(),
                    "right_joint_command_subs": self.pub_pd[RIGHT].get_subscription_count(),
                }
            elif arm_mode == "cartesian":
                status["arms"] = {
                    "ready": self.move_pose_action.server_is_ready(),
                    "move_pose_action": self.move_pose_action.server_is_ready(),
                }
            else:
                status["arms"] = {
                    "ready": (
                        self.trajectory_action[LEFT].server_is_ready()
                        and self.trajectory_action[RIGHT].server_is_ready()
                        and self.gripper_action[LEFT].server_is_ready()
                        and self.gripper_action[RIGHT].server_is_ready()
                    ),
                    "left_trajectory_action": self.trajectory_action[LEFT].server_is_ready(),
                    "right_trajectory_action": self.trajectory_action[RIGHT].server_is_ready(),
                    "left_gripper_action": self.gripper_action[LEFT].server_is_ready(),
                    "right_gripper_action": self.gripper_action[RIGHT].server_is_ready(),
                }
        if "chassis" in components:
            status["chassis"] = {
                "ready": (
                    self.pub_cmd_vel.get_subscription_count() > 0
                    and self.pub_chassis_state.get_subscription_count() > 0
                ),
                "cmd_vel_subs": self.pub_cmd_vel.get_subscription_count(),
                "state_command_subs": self.pub_chassis_state.get_subscription_count(),
            }
        if "lift" in components:
            status["lift"] = {
                "ready": (
                    self.pub_lift_abs.get_subscription_count() > 0
                    and self.pub_lift_vel.get_subscription_count() > 0
                ),
                "move_absolute_subs": self.pub_lift_abs.get_subscription_count(),
                "set_velocity_subs": self.pub_lift_vel.get_subscription_count(),
                "enable_service": self.cli_lift_enable.service_is_ready(),
            }
        if "waist" in components:
            status["waist"] = {
                "ready": (
                    self.pub_waist_pos.get_subscription_count() > 0
                    and self.pub_waist_cmd.get_subscription_count() > 0
                    and self.pub_waist_enable.get_subscription_count() > 0
                ),
                "position_command_subs": self.pub_waist_pos.get_subscription_count(),
                "command_subs": self.pub_waist_cmd.get_subscription_count(),
                "enable_subs": self.pub_waist_enable.get_subscription_count(),
            }
        if "emergency_stop" in components:
            status["emergency_stop"] = {
                "ready": self.cli_estop.service_is_ready(),
                "trigger_service": self.cli_estop.service_is_ready(),
            }
        status["ready"] = all(item["ready"] for item in status.values())
        return status

    # ================= 腰部 =================
    def waist_position(self, position):
        self.pub_waist_pos.publish(Float64(data=float(position)))

    def waist_command(self, position, velocity, torque):
        msg = Float64MultiArray()
        msg.data = [float(position), float(velocity), float(torque)]
        self.pub_waist_cmd.publish(msg)

    def waist_enable(self, enable=True):
        self.pub_waist_enable.publish(Bool(data=bool(enable)))

    # ================= 急停 =================
    def emergency_stop(self, wait_sec: float = 0.0) -> bool:
        """触发 /emergency_stop_node/trigger。

        wait_sec<=0：只发出异步请求（WS 路径）。
        wait_sec>0：spin 等待服务处理完成（关闭路径，需在杀进程前完成 CAN 失能）。
        """
        if not self.cli_estop.service_is_ready():
            return False
        future = self.cli_estop.call_async(Trigger.Request())
        if wait_sec <= 0.0:
            return True
        deadline = time.monotonic() + wait_sec
        while not future.done() and time.monotonic() < deadline:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception:  # noqa: BLE001
                break
        if not future.done():
            return False
        try:
            resp = future.result()
            return bool(resp is not None and resp.success)
        except Exception:  # noqa: BLE001
            return False

    # ================= 状态订阅回调 =================
    def _on_joint_states(self, msg: JointState):
        # 按关节名合并：左右 teleop 各发一侧时不能整包覆盖。
        with self._lock:
            arms = self._state["arms"]
            index = {name: i for i, name in enumerate(arms["name"])}
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                eff = msg.effort[i] if i < len(msg.effort) else 0.0
                if name in index:
                    j = index[name]
                    if j < len(arms["position"]):
                        arms["position"][j] = pos
                    if j < len(arms["velocity"]):
                        arms["velocity"][j] = vel
                    if j < len(arms["effort"]):
                        arms["effort"][j] = eff
                else:
                    arms["name"].append(name)
                    arms["position"].append(pos)
                    arms["velocity"].append(vel)
                    arms["effort"].append(eff)

    def _on_left_joint_command(self, msg: JointState):
        self._on_joint_command(LEFT, msg)

    def _on_right_joint_command(self, msg: JointState):
        self._on_joint_command(RIGHT, msg)

    def _on_joint_command(self, side: str, msg: JointState):
        """缓存 VR/IK 发布的 7 个臂关节和 1 个归一化夹爪目标。"""
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self._lock:
            self._state["teleop_action"][side] = {
                "position": list(msg.position),
                "stamp": stamp,
                "recv_time": time.time(),
            }

    def _on_lift_pos(self, msg):
        with self._lock:
            self._state["lift"]["position"] = msg.data

    def _on_lift_vel(self, msg):
        with self._lock:
            self._state["lift"]["velocity"] = msg.data

    def _on_lift_enabled(self, msg):
        with self._lock:
            self._state["lift"]["enabled"] = msg.data

    def _on_lift_error(self, msg):
        with self._lock:
            self._state["lift"]["error"] = msg.data

    def _on_lift_error_code(self, msg):
        with self._lock:
            self._state["lift"]["error_code"] = msg.data

    def _on_lift_error_message(self, msg):
        with self._lock:
            self._state["lift"]["error_message"] = msg.data

    def _on_lift_status(self, msg):
        with self._lock:
            self._state["lift"]["status"] = msg.data

    def _on_waist_state(self, msg):
        d = list(msg.data)
        with self._lock:
            self._state["waist"] = {
                "position": d[0] if len(d) > 0 else 0.0,
                "velocity": d[1] if len(d) > 1 else 0.0,
                "torque": d[2] if len(d) > 2 else 0.0,
            }

    def _on_chassis_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self._log_error(f"底盘状态 JSON 解析失败: {exc}")
            return
        if not isinstance(status, dict):
            self._log_error("底盘状态必须是 JSON 对象")
            return
        with self._lock:
            self._state["chassis"] = status

    def _on_ee_poses(self, msg: TFMessage):
        """openarm_move_pose/ee_pose_publisher 发布的 TFMessage：按 child_frame_id 拆左右。"""
        ee = {}
        for ts in msg.transforms:
            side = self.ee_link_to_side.get(ts.child_frame_id)
            if side is None:
                continue
            tr = ts.transform
            ee[side] = {
                "position": {
                    "x": tr.translation.x,
                    "y": tr.translation.y,
                    "z": tr.translation.z,
                },
                "orientation": {
                    "x": tr.rotation.x,
                    "y": tr.rotation.y,
                    "z": tr.rotation.z,
                    "w": tr.rotation.w,
                },
            }
        if not ee:
            return
        with self._lock:
            self._state["ee_poses"].update(ee)

    def snapshot(self) -> dict:
        """聚合当前最新状态，拆分左右臂。"""
        with self._lock:
            arms = self._state["arms"]
            lift = dict(self._state["lift"])
            waist = dict(self._state["waist"])
            chassis = dict(self._state["chassis"])
            ee_poses = {k: dict(v) for k, v in self._state["ee_poses"].items()}
            teleop = {side: dict(value) for side, value in self._state["teleop_action"].items()}

        left, right = self._split_arms(arms)
        now = time.time()
        teleop_valid = all(
            len(teleop[side]["position"]) == self.teleop_action_size
            and now - teleop[side]["recv_time"] <= self.teleop_timeout
            for side in (LEFT, RIGHT)
        )
        return {
            "left_arm": left,
            "right_arm": right,
            "lift": lift,
            "waist": waist,
            "chassis": chassis,
            "ee_poses": ee_poses,
            "teleop_action": {
                LEFT: teleop[LEFT]["position"],
                RIGHT: teleop[RIGHT]["position"],
                "stamp": max(teleop[LEFT]["stamp"], teleop[RIGHT]["stamp"]),
                "recv_time": max(teleop[LEFT]["recv_time"], teleop[RIGHT]["recv_time"]),
                "valid": teleop_valid,
            },
        }

    def reset_state(self):
        """清空桥接层缓存的最近一次状态。"""
        with self._lock:
            self._state = {
                "arms": {"name": [], "position": [], "velocity": [], "effort": []},
                "lift": {},
                "waist": {},
                "chassis": {},
                "ee_poses": {},
                "teleop_action": {
                    LEFT: {"position": [], "stamp": 0.0, "recv_time": 0.0},
                    RIGHT: {"position": [], "stamp": 0.0, "recv_time": 0.0},
                },
            }

    def _split_arms(self, arms: dict) -> tuple[dict, dict]:
        names = arms.get("name", [])
        name_to_index = {name: index for index, name in enumerate(names)}

        def extract(side: str) -> dict:
            ordered_names = [*self.arm_joints[side], self.gripper_joints[side]]
            result = {"position": [], "velocity": [], "effort": []}
            for field in result:
                values = arms.get(field, [])
                result[field] = [
                    values[index]
                    for name in ordered_names
                    if (index := name_to_index.get(name)) is not None and index < len(values)
                ]
            return result

        return extract(LEFT), extract(RIGHT)
