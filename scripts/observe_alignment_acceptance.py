#!/usr/bin/env python3
"""Read-only live correspondence sampling; NEVER sends movement commands."""
import argparse
import json
import math
from pathlib import Path
import time

from verify_startup_alignment import assess, status_request, validate_policy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not 1 <= args.duration <= 120:
        parser.error('duration must be between 1 and 120 seconds')
    policy = validate_policy(json.loads(Path(__file__).with_name('startup_alignment_policy.json').read_text()))
    samples = []
    start = time.monotonic()
    next_log = start
    while time.monotonic() - start < args.duration:
        now = time.monotonic()
        try:
            state = status_request()
            result = assess(state, 'following', policy)
            row = dict(elapsed_s=now-start, status=state, assessment=result)
        except (OSError, ValueError, TypeError) as exc:
            row = dict(elapsed_s=now-start, error=str(exc))
        samples.append(row)
        if now >= next_log:
            s = row.get('status', {})
            pairs = row.get('assessment', {}).get('joints', [])
            print(json.dumps(dict(elapsed_s=round(now-start, 1), state=s.get('state'),
                fault_bits=s.get('fault_bits'),
                max_pair_error_deg=max((abs(math.degrees(j['pair_error_rad'])) for j in pairs), default=None),
                issues=row.get('assessment', {}).get('issues', [row.get('error')])), ensure_ascii=False), flush=True)
            next_log = now + 2
        time.sleep(.1)
    report = dict(kind='read_only_hardware_samples', duration_s=time.monotonic()-start,
                  started_unix_s=time.time()-(time.monotonic()-start), samples=samples)
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
