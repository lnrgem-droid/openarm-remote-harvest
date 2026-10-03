# Left master alignment protocol extension

The v3 packet layout and payload sizes remain unchanged. The extension assigns
previously invalid header flag values; it does not make mixed deployments safe.

## Action acknowledgements

Each controller mode is `0` for free, `1` for servo, or `2` for latched failure.
The right mode occupies bits 0–1. When capability mask **`0x10`** is present,
bits 2–3 carry the left mode. The mask is hexadecimal sixteen, not bit index 16.

| Value | Meaning |
| --- | --- |
| `0`, `1`, `2` | Legacy right-only ACK; left mode is **unknown**, not free |
| `0x10` | Both modes explicitly reported free |
| `0x11`, `0x12` | Right servo/failure; left explicitly free |
| `0x14`, `0x18` | Left servo/failure; right explicitly free |

Other combinations of the valid per-side modes can report conflicts, but the
runtime must stop the affected operation and must not treat either side as free.
Mode `3`, missing capability with left-mode bits, and reserved bits are rejected.
`decode_collection_ack()` centralizes this validation.

## Follower targets

Existing flags retain their meaning: `1` holds the left follower, `2` holds the
right follower, and `4` requests the right master servo. New flag `8` requests
the left master servo. The one eight-axis target belongs to the selected master.

Valid new combinations are `9` and `11`: the left follower is always held while
the left master servo is requested. A servo without its follower HOLD, or both
servo bits together, is invalid. Legal existing combinations remain accepted.

Left alignment requires a real capability-bearing ACK and both modes free at
entry. It waits for left servo ACK before advancing the fixed-target trajectory.
Completion withdraws bit `8`, waits for fresh left mode `0`, then rechecks the
eight-axis `0.06 rad` alignment requirement before releasing follower HOLD.
Pause also explicitly withdraws the request and confirms mode `0`; it never
releases follower HOLD. A failed or interrupted path cannot automatically resume.

## Compatibility and deployment preflight

The new decoder still accepts legacy ACKs `0/1/2` and existing follower flags,
but legacy ACKs never enable left master alignment. Old decoders reject new ACK
values and new follower flags. Packet rejection is a failure barrier, not a
supported operating mode: it can trigger network/watchdog holds.

Before starting either gateway, deployment must verify that **every consumer of
the action/state protocol** uses this extension, including the leader, follower,
and recording/preview bridges. Deploy the matching protocol module together
with the gateways; do not activate a new producer against an old decoder. The
physical leader must expose valid left and right collection mode fields before
the left alignment capability is advertised. A left capability flag must never
be synthesized merely from the presence of the Python code.

Regression coverage lives in `test/test_left_master_protocol.py` and the runtime
tests `test_left_master_align.py` and `test_left_master_follower_gate.py`. Tests
cover legacy ACK acceptance, missing-capability start rejection, invalid flag
combinations, side conflicts, eight-axis tracking, release confirmation, and
faults without creating ROS nodes or accessing devices.
