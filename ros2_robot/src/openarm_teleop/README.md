# OpenArm Teleop

Leader → follower teleoperation (unilateral / bilateral). Bilateral control can
publish ROS 2 topics for LeRobot recording via `robot_bridge`.

## LeRobot recording (dual-arm bilateral)

Do **not** start `openarm_gravity_pd_control` at the same time (same CAN).

```bash
cd ~/openarm_robot/ros2_robot
source /opt/ros/humble/setup.bash
colcon build --packages-up-to openarm_teleop robot_bridge
source install/setup.bash

# terminal 1: bilateral force teleop (publishes /joint_states and joint_command)
bash src/openarm_teleop/script/launch_bimanual_bilateral.sh

# terminal 2: bridge only (does not start PD)
ros2 launch robot_bridge bridge.launch.py arm_mode:=gravity_pd
```

Then run `lerobot-record` with `--robot.control_authority=external` and
`--teleop.type=openarm_bridge_teleop`.

CAN defaults: right follower `can0` / leader `can2`; left follower `can1` / leader `can3`.

Inference uses `openarm_gravity_pd_control` instead of this package.

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/teleop/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See [LICENSE.txt](LICENSE.txt) for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).


## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/teleop/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See [LICENSE.txt](LICENSE.txt) for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
