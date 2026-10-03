#!/usr/bin/env python3
"""Validate the complete dual-arm RUNNING contract from follower status JSON."""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
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


def is_healthy_running(
    status: dict[str, Any], max_tracking_error_rad: float | None = None
) -> bool:
    """Control stack is healthy; individual HOLD/RETURNING modes still apply."""
    age = float(status.get("action_age_ms", float("inf")))
    healthy = (
        status.get("state") == "RUNNING"
        and int(status.get("fault_bits", -1)) == 0
        and status.get("relative_follow_reference_captured") is True
        and set(status.get("enabled_arms", [])) == {"left", "right"}
        and status.get("leader_session_id") not in (None, 0)
        and status.get("feedback_fresh_for_control") is True
        and math.isfinite(age) and 0.0 <= age <= 100.0
    )
    if not healthy or max_tracking_error_rad is None:
        return healthy
    # A controller left alive across motor power cycling can keep publishing a
    # perfectly fresh RUNNING heartbeat even though the drives lost their
    # enable/session state.  Reuse is allowed only while both followers are
    # still physically tracking their targets closely.
    return (
        float(status.get("max_tracking_error_rad", float("inf")))
        <= max_tracking_error_rad
        and float(status.get("left_max_tracking_error_rad", float("inf")))
        <= max_tracking_error_rad
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-tracking-error-rad", type=float)
    parser.add_argument("--startup-report", help="require successful acceptance for this leader session")
    parser.add_argument("--health-report", help="require fresh session-matched physical motor readiness")
    parser.add_argument("--explain", action="store_true", help="print the actual rejected gates")
    args = parser.parse_args()
    issues = []
    try:
        status = parse_status(sys.stdin.read())
        if args.health_report:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from teleop_readiness import apply_current_report
            checked = apply_current_report(status, args.health_report)
            if not checked.get('teleop_ready'):
                issues.append('电机健康检查 [' + str(checked.get('state')) + ']：' +
                              checked.get('readiness_reason', '未知原因') +
                              '；证据：' + args.health_report)
        if args.startup_report:
            try:
                report = json.loads(Path(args.startup_report).read_text())
                if not (
                    report.get("accepted") is True
                    and report.get("phase") == "following"
                    and report.get("leader_session_id") == status.get("leader_session_id")
                ):
                    issues.append('启动对齐验收未通过或不属于当前主臂会话：' + args.startup_report)
            except (OSError, ValueError, AttributeError) as exc:
                issues.append('无法读取启动对齐验收：' + str(exc))
        if not is_healthy_running(status, args.max_tracking_error_rad):
            issues.append('控制状态/会话/跟踪误差未通过：state=' + str(status.get('state')) +
                          '，fault_bits=' + str(status.get('fault_bits')) +
                          '，reason=' + str(status.get('reason', '')))
    except (OSError, TypeError, ValueError, AttributeError) as exc:
        issues.append('无法验证控制状态：' + str(exc))
    if args.explain:
        for issue in issues:
            print('  - ' + issue, file=sys.stderr)
    return 1 if issues else 0


if __name__ == "__main__":
    raise SystemExit(main())
