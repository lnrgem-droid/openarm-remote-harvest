"""Strict local Unix-datagram protocol between controller and watchdog process."""

from __future__ import annotations

import json
from typing import Union

from .watchdog import ControllerHeartbeat


SCHEMA_VERSION = 1
MAX_LOCAL_DATAGRAM = 4096
COMMANDS = {"alignment_complete", "request_run", "hold", "reset", "estop", "status"}


class LocalProtocolError(ValueError):
    pass


def encode_heartbeat(heartbeat: ControllerHeartbeat) -> bytes:
    return _dump(
        {
            "schema_version": SCHEMA_VERSION,
            "type": "heartbeat",
            "controller_session_id": heartbeat.controller_session_id,
            "sequence": heartbeat.sequence,
            "sent_monotonic_ns": heartbeat.sent_monotonic_ns,
            "last_control_cycle_ns": heartbeat.last_control_cycle_ns,
            "last_action_rx_ns": heartbeat.last_action_rx_ns,
            "leader_session_id": heartbeat.leader_session_id,
            "can_ok": heartbeat.can_ok,
            "estop_active": heartbeat.estop_active,
            "consecutive_overruns": heartbeat.consecutive_overruns,
        }
    )


def encode_command(command: str, **fields: Union[int, bool]) -> bytes:
    if command not in COMMANDS:
        raise LocalProtocolError(f"unknown command {command}")
    return _dump(
        {
            "schema_version": SCHEMA_VERSION,
            "type": "command",
            "command": command,
            **fields,
        }
    )


def decode_local(datagram: bytes) -> tuple[str, Union[ControllerHeartbeat, dict]]:
    if not datagram or len(datagram) > MAX_LOCAL_DATAGRAM:
        raise LocalProtocolError("invalid local datagram size")
    try:
        value = json.loads(datagram.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LocalProtocolError("invalid local JSON") from exc
    if not isinstance(value, dict):
        raise LocalProtocolError("local datagram must be an object")
    if value.get("schema_version") != SCHEMA_VERSION:
        raise LocalProtocolError("unsupported local schema version")
    kind = value.get("type")
    if kind == "heartbeat":
        expected = {
            "schema_version",
            "type",
            "controller_session_id",
            "sequence",
            "sent_monotonic_ns",
            "last_control_cycle_ns",
            "last_action_rx_ns",
            "leader_session_id",
            "can_ok",
            "estop_active",
            "consecutive_overruns",
        }
        if set(value) != expected:
            raise LocalProtocolError("heartbeat fields do not match schema v1")
        if type(value["can_ok"]) is not bool or type(value["estop_active"]) is not bool:
            raise LocalProtocolError("heartbeat boolean fields must be JSON booleans")
        try:
            heartbeat = ControllerHeartbeat(
                controller_session_id=value["controller_session_id"],
                sequence=value["sequence"],
                sent_monotonic_ns=value["sent_monotonic_ns"],
                last_control_cycle_ns=value["last_control_cycle_ns"],
                last_action_rx_ns=value["last_action_rx_ns"],
                leader_session_id=value["leader_session_id"],
                can_ok=value["can_ok"],
                estop_active=value["estop_active"],
                consecutive_overruns=value["consecutive_overruns"],
            )
        except (TypeError, ValueError) as exc:
            raise LocalProtocolError(str(exc)) from exc
        return kind, heartbeat
    if kind == "command":
        command = value.get("command")
        if command not in COMMANDS:
            raise LocalProtocolError("unknown local command")
        common = {"schema_version", "type", "command"}
        if command in ("alignment_complete", "request_run"):
            if set(value) != common | {"leader_session_id"}:
                raise LocalProtocolError(f"{command} fields do not match schema v1")
            _require_nonzero_uint64(value["leader_session_id"], "leader_session_id")
        elif command == "reset":
            if set(value) != common | {"estop_released"}:
                raise LocalProtocolError("reset fields do not match schema v1")
            if type(value["estop_released"]) is not bool:
                raise LocalProtocolError("estop_released must be a JSON boolean")
        elif set(value) != common:
            raise LocalProtocolError("estop fields do not match schema v1")
        return kind, value
    raise LocalProtocolError("unknown local message type")


def _dump(value: dict) -> bytes:
    raw = json.dumps(value, separators=(",", ":"), sort_keys=True).encode("utf-8")
    if len(raw) > MAX_LOCAL_DATAGRAM:
        raise LocalProtocolError("local datagram is too large")
    return raw


def _require_nonzero_uint64(value: object, field: str) -> None:
    if type(value) is not int or not 0 < value <= 0xFFFFFFFFFFFFFFFF:
        raise LocalProtocolError(f"{field} must be a nonzero uint64")
