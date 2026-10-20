# Temporary laptop leader + desktop follower capture test

These are the deployment files used on 2026-10-20 while the Jetson was unavailable. The laptop controls the two leader arms on `can0`/`can1`; the old desktop controls the follower arms on `can0`/`can1` and serves three RGB-D cameras. The normal Jetson follower mapping remains `can1`/`can2` in the shared startup scripts.

- On the old desktop, install the three `openarm-host-*.service` capture units as user units, and keep `prepare-host-capture-services.sh` and `start-host-record-bridge.sh` at `/home/openarm/openarm_robot/`.
- On the laptop, install `openarm-host-test-teleop.service` under `~/.config/systemd/user/`, the two laptop launcher scripts under `/home/openarm/openarm-transfer/`, and the desktop entry under `~/桌面/`.
- Opening the launcher prepares CAN/capture services and opens the console. It does not enable or home the arms. The console's explicit start button requires onsite confirmation before enabling and homing all four arms. Recording starts only when the operator presses its record button.

The files contain machine-specific absolute paths and wired addresses. Review them before using this arrangement on another machine. The verified source code lives in this repository and the `feat/jetson-rgbd-preview` branch of the same GitHub repository; build artifacts, credentials, and recorded datasets are intentionally absent.
