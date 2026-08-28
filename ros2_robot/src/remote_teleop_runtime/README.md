# Remote teleoperation runtime (v1)

The right-arm launch configuration remains deliberately **right-arm only**.  The
separate bimanual launch configuration enables both arms after the single-arm
configuration has been verified:

- x86 leader: right `can0`, left `can1`
- Jetson follower: right `can1`, left `can2`
- UDP action/state: `50010/50011`, 100 Hz
- follower CAN loop: 500 Hz
- left arm is disabled and is never initialized

Startup holds each measured pose; it never homes automatically. The follower applies
remote targets only while the independent watchdog reports `RUNNING`. `hold`, stale
action data, stale watchdog permission, and faults all publish the latest measured pose.

The watchdog must be started separately. Use
`ros2 run remote_teleop_runtime remote-teleop-control status|align|run|hold|reset|disable`
on the Jetson. `disable`
is only valid after an operator supports the arm; it latches E-stop and calls the
controller disable service.

Launch commands and the supervised physical sequence are documented in
`docs/RIGHT_ARM_REAL_TEST.md` at the repository root.

## One-command dual-machine startup

After both host and Jetson workspaces have been built from the same release,
run this **on the host**:

```bash
/home/openarm/dev/openarm-remote-harvest/scripts/run_bimanual_remote_feedback.sh
```

The script starts the Jetson follower stack through SSH, returns both pairs to
their existing calibrated encoder `q=0` references, waits for live UDP traffic,
then requests `align` and `run`. It never relaxes the watchdog alignment gate:
if alignment is rejected, no teleoperation starts. `Ctrl+C` sends `hold`; only
after an operator supports the arms should `disable` be issued on the Jetson.
