# Jetson offline update, 2026-10-20

This directory backs up the updater deployed on the laptop. The complete offline
bundle, including the two source archives, is attached to the GitHub release
`field-update-20261020`. The archives are pinned by `manifest.json`; they contain
the already-published control commit `f1a536c` and capture commit `691dc6b`.
Deployment-only changes made after those commits do not require rebuilding the
robot control payload.

Extract the release bundle into `/home/openarm/openarm-transfer/` on the configured
laptop, retaining the `jetson-update-20261020` directory name. Copy its
`更新Jetson采集程序.desktop` to the desktop and allow launching it. The existing SSH
alias `openarm-jetson` must point to the previously paired Jetson; keys and SSH
credentials are deliberately not included.

Start the Jetson, connect the Ethernet cable, and run the update before opening
the collection console. The updater refuses active control/capture processes,
checks payload hashes and the expected `nvidia`/`aarch64` target, builds in a new
directory using the existing ROS Humble dependencies, preserves current control
parameters, and verifies imports and native-library linkage before activation.
It keeps the previous installation in a backup and journals changes for rollback.
It neither enables motors nor starts cameras or teleoperation.

The permanent build/install location is under
`/home/nvidia/openarm-offline-updates/job-…/`; do not delete it after installation.
The local terminal retains the log. If SSH disconnects, the remote supervisor
continues; relaunching the same desktop entry resumes reading that job.

Validation: 17 hardware-free tests, an x86 native build, staged import/library
checks, and a simulated existing-overlay activation passed. Jetson was offline
when this bundle was prepared: actual ARM installation, camera streams, and
recorded dataset completeness still require onsite verification. Software
installation success is not a hardware acceptance result.

Run the regression tests with:

```bash
python3 deployment/jetson-offline-update/test_installer.py
```

The separate `../field-ready/` directory contains the read-only connection/source
checker and field instructions. Source comparison alone does not verify an
installed binary or camera health.
