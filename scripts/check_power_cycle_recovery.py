#!/usr/bin/env python3
"""Classify only the all-drives-disabled, idle, unfaulted recovery case."""
import json
import sys


def can_reinitialize(status, motors):
    collection = status.get('collection') or {}
    return (
        motors.get('result') == 'DISABLED'
        and status.get('state') == 'RUNNING'
        and status.get('fault_bits') == 0
        and status.get('feedback_fresh_for_control') is True
        and set(status.get('enabled_arms', [])) == {'left', 'right'}
        and collection.get('left_mode') == 'FOLLOW'
        and collection.get('right_mode') == 'FOLLOW'
        and collection.get('recording', 'unknown') is None
        and collection.get('return_phase') == 'idle'
        and collection.get('transitioning_arms') == []
    )


if __name__ == '__main__':
    try:
        status, motors = json.load(sys.stdin)
        sys.exit(0 if can_reinitialize(status, motors) else 1)
    except (ValueError, TypeError, AttributeError):
        sys.exit(1)
