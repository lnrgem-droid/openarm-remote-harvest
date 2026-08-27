# Remote teleoperation runtime (v1)

The checked-in launch configuration is deliberately **right-arm only**:

- x86 leader: `can0`
- Jetson follower: `can1`
- UDP action/state: `50010/50011`, 100 Hz
- follower CAN loop: 500 Hz
- left arm is disabled and is never initialized

Startup holds the measured pose; it never homes automatically. The follower applies
remote targets only while the independent watchdog reports `RUNNING`. `hold`, stale
action data, stale watchdog permission, and faults all publish the latest measured pose.

The watchdog must be started separately. Use
`remote-teleop-control status|align|run|hold|reset|disable` on the Jetson. `disable`
is only valid after an operator supports the arm; it latches E-stop and calls the
controller disable service.

Launch commands and the supervised physical sequence are documented in
`docs/RIGHT_ARM_REAL_TEST.md` at the repository root.
