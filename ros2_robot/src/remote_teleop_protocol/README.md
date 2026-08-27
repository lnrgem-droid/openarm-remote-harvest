# OpenArm remote teleoperation protocol v1

This package freezes the first host-to-Jetson interface contract. It contains no
ROS node, OpenArm driver, CAN access, motor enable, or motion command path.

## Transport and packet rules

- High-rate action and follower-state traffic uses UDP over the dedicated LAN.
- Every datagram is big-endian, versioned, CRC32 protected, and smaller than 1200 bytes.
- A nonzero random 64-bit `session_id` changes on each process start. `sequence`
  starts at 1 and increases within one session. Receivers reject duplicate,
  out-of-order, and unexpected-session packets. A session change is accepted only
  after the local state machine has left `RUNNING`, entered `ALIGNING`, and explicitly
  reset its sequence tracker.
- All transmitted timestamps are unsigned nanoseconds from the producing host's
  monotonic clock. Values from different hosts are different clock domains and
  must not be directly subtracted.
- The initial rates and timeouts in `config/remote_teleop_v1.yaml` are test values.
  The final local control period follows measured sustainable performance; 1 kHz
  is a target, not an assumption.

## Axis order and units

Every action/state axis vector has exactly 16 entries:

```text
left_joint1..left_joint7, left_gripper,
right_joint1..right_joint7, right_gripper
```

Arm and gripper positions are radians; velocities are radians/second. The protocol
does not normalize the gripper to 0..1. A calibrated gripper mapping belongs in the
hardware adapter and must be measured before motor control is connected.

## Messages

`ACTION` contains the 16 desired positions plus `valid_for_ns`. Receipt of a valid
packet does not authorize motion: the future follower controller must additionally
be in `RUNNING`, aligned, locally healthy, and accepted by its independent watchdog.
The follower starts the validity timer from its own local monotonic receive time and
uses the smaller of packet `valid_for_ns` and its configured local maximum. It never
subtracts a leader timestamp from a Jetson timestamp.

`FOLLOWER_STATE` contains 16 observed positions, 16 observed velocities,
`obs_timestamp_ns`, `action_timestamp_ns`, `applied_action_session_id`,
`applied_action_sequence`, control state, and fault bits. A zero action timestamp,
applied session, and applied sequence mean that no remote action has entered a
control cycle; these three fields must be zero or nonzero together.

CRC32 detects accidental corruption but is not authentication. Protocol v1 assumes
a physically controlled dedicated LAN; firewalling and authenticated supervisory
access are separate deployment requirements.

Control states are `INIT`, `ALIGNING`, `READY`, `RUNNING`, `FAULT`, and `E_STOP`.
After every process restart the follower starts outside `RUNNING`.

## Time contract for recording

- `obs_timestamp_ns`: Jetson monotonic time at the actual follower-feedback read.
- `action_timestamp_ns`: Jetson monotonic time when the applied action enters the
  local control cycle, never the network-receive time.
- `camera_device_timestamp`: original camera device time and its clock-domain ID.
- `camera_host_timestamp_ns`: Jetson monotonic time when the frame is received.
- Recorder receipt time is diagnostic only and never replaces production time.
- The OpenArm raw layer preserves native stream rates. LeRobot conversion samples
  at 30 Hz: qpos may interpolate, action uses zero-order hold, and RGB/depth use the
  nearest frame. Pairing tolerances are measured in phase 0; over-limit samples are
  marked or dropped, never silently paired.

The camera shared-memory contract will separately define frame number, both
timestamps, clock domain, dimensions, pixel format, stride, serial number, buffer
generation, and buffer lifetime before camera recording code is integrated.

## CAN-free simulation

The installed simulation executables only import Python socket/protocol modules.
They cannot access CAN or OpenArm hardware. Their ports (`51010/51011`) intentionally
differ from the reserved control ports (`50010/50011`).

```bash
remote-teleop-sim-follower --duration 10
remote-teleop-sim-leader --peer 127.0.0.1 --duration 5
```

For a two-computer test, start the simulator follower on Jetson, then set the
leader peer to `192.168.50.2`. This still performs network-only zero-vector traffic.
