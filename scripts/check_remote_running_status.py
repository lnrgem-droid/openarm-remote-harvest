#!/usr/bin/env python3
"""Validate the complete dual-arm RUNNING contract from follower status JSON."""
from __future__ import annotations

import json
import sys
from typing import Any


def parse_status(text: str) -> dict[str, Any]:
    """Accept a JSON object even if ROS emits a harmless prefix/suffix line."""
    decoder = json.JSONDecoder()
    for index, character in enumerate(text):
        if character != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    raise ValueError("no follower status JSON object")


def is_healthy_running(status: dict[str, Any]) -> bool:
    """True only when moving a leader is expected to move both followers."""
    return (
        status.get("state") == "RUNNING"
        and int(status.get("fault_bits", -1)) == 0
        and status.get("relative_follow_reference_captured") is True
        and set(status.get("enabled_arms", [])) == {"left", "right"}
        and status.get("leader_session_id") not in (None, 0)
        and status.get("feedback_fresh_for_control") is True
        and float(status.get("action_age_ms", float("inf"))) <= 100.0
    )


def main() -> int:
    try:
        status = parse_status(sys.stdin.read())
        return 0 if is_healthy_running(status) else 1
    except (TypeError, ValueError):
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
