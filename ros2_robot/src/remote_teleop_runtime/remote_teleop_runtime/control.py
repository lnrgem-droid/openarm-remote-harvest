import argparse, json, os, socket, tempfile
from .common import RUNTIME_SOCKET

def main():
    parser = argparse.ArgumentParser(description="Jetson-local follower control")
    parser.add_argument("command", choices=("status", "align", "run", "hold", "reset", "disable"))
    args = parser.parse_args()
    path = tempfile.mktemp(prefix="openarm_ctl_", dir="/tmp")
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM); sock.bind(path); sock.settimeout(2.0)
    try:
        sock.sendto(json.dumps({"command": args.command}).encode(), RUNTIME_SOCKET)
        result = json.loads(sock.recv(4096).decode()); print(json.dumps(result, indent=2, ensure_ascii=False))
        if "error" in result: raise SystemExit(2)
    finally:
        sock.close()
        if os.path.exists(path): os.unlink(path)
