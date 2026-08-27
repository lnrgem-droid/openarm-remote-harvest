import json

import pytest

from remote_teleop_follower_safety.local_protocol import (
    LocalProtocolError,
    decode_local,
    encode_command,
    encode_heartbeat,
)
from remote_teleop_follower_safety.watchdog import ControllerHeartbeat


def sample_heartbeat():
    return ControllerHeartbeat(1, 2, 100, 99, 98, 3, True, False, 0)


def test_heartbeat_round_trip():
    kind, decoded = decode_local(encode_heartbeat(sample_heartbeat()))
    assert kind == "heartbeat"
    assert decoded == sample_heartbeat()


def test_command_round_trip():
    kind, decoded = decode_local(encode_command("alignment_complete", leader_session_id=3))
    assert kind == "command"
    assert decoded["leader_session_id"] == 3


def test_unknown_or_extra_heartbeat_fields_are_rejected():
    raw = json.loads(encode_heartbeat(sample_heartbeat()))
    raw["unexpected"] = 1
    with pytest.raises(LocalProtocolError, match="fields"):
        decode_local(json.dumps(raw).encode())


def test_boolean_integer_is_rejected():
    raw = json.loads(encode_heartbeat(sample_heartbeat()))
    raw["can_ok"] = 1
    with pytest.raises(LocalProtocolError, match="booleans"):
        decode_local(json.dumps(raw).encode())


@pytest.mark.parametrize("bad_value", ["false", 0, 1, None])
def test_reset_release_requires_real_boolean(bad_value):
    raw = {
        "schema_version": 1,
        "type": "command",
        "command": "reset",
        "estop_released": bad_value,
    }
    with pytest.raises(LocalProtocolError, match="JSON boolean"):
        decode_local(json.dumps(raw).encode())


def test_command_extra_fields_are_rejected():
    raw = json.loads(encode_command("estop"))
    raw["estop_released"] = True
    with pytest.raises(LocalProtocolError, match="fields"):
        decode_local(json.dumps(raw).encode())
