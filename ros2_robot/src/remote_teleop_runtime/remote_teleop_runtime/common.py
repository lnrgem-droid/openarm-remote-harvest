from __future__ import annotations

import json
import os
import socket
import tempfile
import time

from remote_teleop_follower_safety.local_protocol import encode_command

WATCHDOG_SOCKET = "/tmp/openarm_follower_watchdog.sock"
RUNTIME_SOCKET = "/tmp/openarm_remote_runtime.sock"
ACTION_PORT = 50010
STATE_PORT = 50011
GRIPPER_OPEN_M = 0.044
GRIPPER_MAX_RAD = -1.0472

# ROS remains strictly local to each computer.  These role-qualified names are
# a second barrier against accidentally mixing leader feedback with follower
# feedback if a launch environment is later misconfigured.
LEADER_JOINT_STATES_TOPIC = "/leader/joint_states"
FOLLOWER_JOINT_STATES_TOPIC = "/follower/joint_states"
LEADER_RIGHT_COMMAND_TOPIC = "/leader/right_arm/joint_command"
FOLLOWER_RIGHT_COMMAND_TOPIC = "/follower/right_arm/joint_command"
FOLLOWER_DISABLE_SERVICE = "/follower/openarm_gravity_pd/disable"


class UnixDatagramClient:
    def __init__(self, server: str):
        self.server = server
        self.path = tempfile.mktemp(prefix="openarm_rt_", dir="/tmp")
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(self.path)
        self.sock.setblocking(False)

    def close(self):
        self.sock.close()
        if os.path.exists(self.path):
            os.unlink(self.path)

    def exchange(self, payload: bytes, timeout_s: float = 0.05) -> dict | None:
        try:
            self.sock.sendto(payload, self.server)
        except OSError:
            return None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                return json.loads(self.sock.recv(4096).decode("utf-8"))
            except BlockingIOError:
                time.sleep(0.001)
        return None


def safety_command(client: UnixDatagramClient, command: str, **fields) -> dict | None:
    return client.exchange(encode_command(command, **fields))
