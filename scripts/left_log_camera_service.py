#!/usr/bin/env python3
"""Same camera owner and format, isolated marker/ports, chest + left wrist only."""
import jetson_orbbec_rgbd_service as camera
from left_log_common import ACTIVE,ERROR,CAMERA_STATUS
camera.ACTIVE_MARKER=ACTIVE
camera.ERROR_MARKER=ERROR
camera.STATUS_FILE=CAMERA_STATUS
# Retain the global physical-camera lock: never open a second SDK owner.
if __name__=='__main__': camera.main()
