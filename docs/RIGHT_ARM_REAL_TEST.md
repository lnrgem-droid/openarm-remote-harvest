# Right-arm remote teleoperation admission test

This is the mandatory gate for the first physical test. Left-arm launch is not
included in v1 and must remain disconnected from the running processes.

## Fixed mapping

- x86 leader right arm: `can2`, host `192.168.50.1`
- Jetson follower right arm: `can1`, host `192.168.50.2`
- action/state UDP: `50010/50011`
- action network rate: 100 Hz; follower CAN loop: 500 Hz
- J1-J4 limit: 0.15 rad/s; J5-J7 limit: 0.25 rad/s
- remote target is clamped to 0.20 rad from the measured follower pose

## Safety rules

Two people are required. One operates the leader; the other supports the follower
and guards the emergency stop. A software hold cannot protect an arm when the
Jetson loses power. Never use Ctrl-C as the normal stop sequence.

Normal stop is: `hold`, physically support the follower, then `disable`.

## Physical admission sequence

1. Configure and verify only the required CAN interface on each host.
2. Start the Jetson follower with `reaction_verified:=false`. It enables only
   `can1`, holds the measured pose and refuses `run`.
3. Start the x86 leader. It enables only `can2` in gravity-compensation mode.
4. On the Jetson, check `remote-teleop-control status`.
5. With the helper supporting the follower, pause MIT refresh for 300 ms, resume,
   inspect joint state, then repeat for 1 s. The service automatically resumes at
   1.2 s even if the terminal command is interrupted.
6. If the arm drops, loses support, oscillates or heats abnormally, execute the
   physical emergency stop and do not continue.
7. Stop both launch processes using the normal stop sequence. Restart the follower
   with `reaction_verified:=true` only after step 5 has passed.
8. Place leader and follower within 0.15 rad on every right-arm joint and keep them
   there for at least one second. Execute `align`, verify `READY`, then execute `run`.
9. Test one joint at a time at +/-0.05 rad, then +/-0.10 rad. Test the gripper at
   25%, 50% and full travel, three cycles each.
10. Test leader process stop, cable removal and follower gateway stop separately.
    A fault must latch; recovery always requires `reset`, alignment and explicit run.

Record direction, maximum tracking error, action acknowledgement RTT p99, fault
reaction time, CAN errors and watchdog trips. Ten minutes of low-speed operation is
required before this file may be copied into a dated right-arm validation report.
