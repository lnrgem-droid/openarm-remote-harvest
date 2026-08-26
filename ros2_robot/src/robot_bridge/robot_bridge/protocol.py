"""JSON 通信协议定义。

纯逻辑模块：只负责帧的构造、解析、校验，不依赖 ROS 或 WebSocket。
所有帧统一结构：
    {
        "type": "command|request|response|state|event|error",
        "seq": <int>,            # 发送方自增序号
        "time": <float>,         # 发送方时间戳，秒
        ...各 type 专有字段
    }
"""

import json
import time


# ---- 帧类型 ----
TYPE_COMMAND = "command"      # 外部 -> 桥接，50Hz 运动控制
TYPE_REQUEST = "request"      # 外部 -> 桥接，一次性请求（启动、急停）
TYPE_RESPONSE = "response"    # 桥接 -> 外部，请求应答
TYPE_STATE = "state"          # 桥接 -> 外部，状态反馈
TYPE_EVENT = "event"          # 桥接 -> 外部，异步事件
TYPE_ERROR = "error"          # 桥接 -> 外部，错误

# ---- 双臂控制模式 ----
ARM_TRAJECTORY = "trajectory"   # ros2_control 轨迹控制
ARM_GRAVITY_PD = "gravity_pd"   # PD + 重力补偿直驱
ARM_CARTESIAN = "cartesian"     # 笛卡尔控制（桥接层做 IK/servo）
ARM_MODES = (ARM_TRAJECTORY, ARM_GRAVITY_PD, ARM_CARTESIAN)


def encode(frame: dict) -> str:
    """dict -> JSON 字符串。"""
    return json.dumps(frame, separators=(",", ":"))


def decode(raw: str) -> dict:
    """JSON 字符串 -> dict，失败抛 ValueError。"""
    try:
        frame = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"非法 JSON: {e}") from e
    if not isinstance(frame, dict):
        raise ValueError("帧必须是 JSON 对象")
    if "type" not in frame:
        raise ValueError("帧缺少 type 字段")
    return frame


def make_response(req: dict, ok: bool, message: str = "", data: dict | None = None) -> dict:
    """根据请求帧构造应答帧，回填 id/seq 便于外部匹配。"""
    return {
        "type": TYPE_RESPONSE,
        "time": time.time(),
        "id": req.get("id"),
        "seq": req.get("seq"),
        "action": req.get("action"),
        "ok": ok,
        "message": message,
        "data": data or {},
    }


def make_event(event: str, data: dict | None = None, req_id=None) -> dict:
    return {
        "type": TYPE_EVENT,
        "time": time.time(),
        "id": req_id,
        "event": event,
        "data": data or {},
    }


def make_error(message: str, code: str = "ERROR", req: dict | None = None) -> dict:
    return {
        "type": TYPE_ERROR,
        "time": time.time(),
        "id": (req or {}).get("id"),
        "seq": (req or {}).get("seq"),
        "code": code,
        "message": message,
    }


def make_state(data: dict, seq: int) -> dict:
    return {
        "type": TYPE_STATE,
        "time": time.time(),
        "seq": seq,
        "data": data,
    }
