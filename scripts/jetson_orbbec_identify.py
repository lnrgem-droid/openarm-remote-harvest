#!/usr/bin/env python3
"""Save one labelled RGB JPEG per Orbbec serial for wrist-role commissioning."""
from __future__ import annotations
import argparse, os
import cv2, numpy as np
from pyorbbecsdk import Config, Context, OBFormat, OBSensorType, Pipeline

def main():
    p=argparse.ArgumentParser(); p.add_argument("--output-dir", required=True); a=p.parse_args(); os.makedirs(a.output_dir, exist_ok=True)
    devices=Context().query_devices()
    for i in range(devices.get_count()):
        device=devices.get_device_by_index(i); serial=device.get_device_info().get_serial_number(); pipe=Pipeline(device); cfg=Config()
        profile=pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_video_stream_profile(640,480,OBFormat.RGB,30)
        cfg.enable_stream(profile); pipe.start(cfg)
        try:
            frame=pipe.wait_for_frames(2000).get_color_frame()
            image=np.frombuffer(frame.get_data(),np.uint8).reshape(frame.get_height(),frame.get_width(),3)
            cv2.imwrite(os.path.join(a.output_dir,f"{serial}.jpg"),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            print(f"saved {serial}.jpg")
        finally: pipe.stop()
if __name__ == "__main__": main()
