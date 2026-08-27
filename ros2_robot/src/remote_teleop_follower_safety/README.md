# Jetson follower safety skeleton

This package provides the first hardware-independent safety layer for the two
follower/harvesting arms connected to the Jetson. It does not import OpenArm, open
CAN, enable motors, write torque, or choose a physical failure reaction.

## Process boundary

`remote-teleop-follower-watchdog` is a separate local process from the future
`follower_control` process and from the UDP network receiver. The controller sends
strict versioned heartbeats over a permission-restricted Unix datagram socket. The
watchdog uses its own Jetson monotonic receive time and detects:

- controller process heartbeat timeout or crash (specific fault bit 6 plus watchdog bit 4);
- stale local control-cycle time in otherwise fresh process heartbeats (specific
  fault bit 7 plus watchdog bit 4);
- leader action receive timeout while `RUNNING`;
- controller-reported CAN failure;
- repeated control-loop overruns;
- emergency stop; and
- unexpected controller or leader session changes.

Faults are latched. Recovery is never automatic. Reset returns to `ALIGNING`, clears
the leader session, and requires fresh alignment plus an explicit RUN request.

## State transitions

```text
INIT --boot--> ALIGNING --alignment_complete--> READY --explicit_run--> RUNNING
                       ^                                      |
                       |                                      v
                explicit reset <------- FAULT / E_STOP <--- any trip
```

A process restart always executes `INIT -> ALIGNING`; it never restores `RUNNING`.
An emergency stop cannot reset until explicitly reported released.

## Deliberate production block

`config/follower_safety_v1.yaml` sets the reaction to `undecided`, marks it
unverified, and disables the hardware backend. Under that configuration the state
machine rejects `READY -> RUNNING`. This is intentional.

Position hold, controlled unload, and drive disable are only named candidates. None
is declared safe until physical tests cover controller SIGKILL, Jetson hang/power
loss, CAN adapter disconnect, CAN cable disconnect, and loss of command refresh.
Disabling torque by default is prohibited because gravity may drop an arm.

The current service is report-only (`hardware_action=REPORT_ONLY_NO_CAN`). It proves
failure detection and lifecycle behavior, not physical stopping. A separate hardware
or drive-level protection is still required for failures where Jetson itself dies.
`systemd` restart may restore availability later but is not a safety mechanism; every
restart must remain in `ALIGNING`.

## CAN-free simulation

For software-only testing, the service can be given a simulated verified reaction.
That option never changes the production configuration and never accesses hardware:

```bash
remote-teleop-follower-watchdog \
  --socket /tmp/openarm_follower_watchdog.sock \
  --duration 3 \
  --simulation-verified-reaction position_hold

remote-teleop-follower-safety-sim \
  --socket /tmp/openarm_follower_watchdog.sock \
  --duration 1 \
  --request-run \
  --fault network_timeout
```

The service output must show `ALIGNING`, `READY`, `RUNNING`, then a latched `FAULT`
with the corresponding bit. These commands are not a real-arm test procedure.
