#!/usr/bin/env python3
"""Host Rerun viewer for the best-effort three-camera RGB preview."""
from __future__ import annotations
import argparse, base64, json, time
import cv2, numpy as np, zmq

def main():
    p=argparse.ArgumentParser(); p.add_argument("--jetson", default="192.168.50.2"); p.add_argument("--port", type=int, default=5556); a=p.parse_args()
    import rerun as rr
    # A named sequence timeline makes the viewer's right edge unambiguously
    # represent the newest Jetson frame instead of a timeless static image.
    rr.init("openarm-rgb-preview", spawn=True)
    ctx=zmq.Context(); sock=ctx.socket(zmq.SUB); sock.setsockopt_string(zmq.SUBSCRIBE, ""); sock.setsockopt(zmq.CONFLATE, 1); sock.connect(f"tcp://{a.jetson}:{a.port}")
    while True:
        data=json.loads(sock.recv_string()); now=time.time()
        for role, encoded in data["images"].items():
            image=cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(encoded),np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
            rr.set_time("preview_sequence", sequence=int(data["frame_seq"][role]))
            rr.set_time("preview_time", timestamp=now)
            rr.log(f"cameras/{role}/rgb", rr.Image(image).compress())
            rr.log(f"cameras/{role}/age_ms", rr.Scalars((now-data["timestamps"][role])*1000))
            rr.log(f"cameras/{role}/sequence", rr.Scalars(data["frame_seq"][role]))
if __name__ == "__main__": main()
