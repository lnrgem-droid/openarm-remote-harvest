#!/usr/bin/env python3
"""Read follower status without loading ROS; never send a motion command."""
import argparse
import json
import subprocess


def remote_source(runtime_socket="/tmp/openarm_remote_runtime.sock", timeout=2.0):
    # The socket and timeout parameters support local fake-server tests. The
    # command-line interface always uses the production path and deadline.
    return '''import json, socket, tempfile
with tempfile.TemporaryDirectory(prefix="oa_start_status_") as directory:
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
        client.bind(directory + "/reply")
        client.settimeout(TIMEOUT)
        client.sendto(b'{"command":"status"}', SOCKET_PATH)
        reply = json.loads(client.recv(65536).decode())
        if not isinstance(reply, dict):
            raise ValueError("runtime status must be an object")
        if "error" not in reply and (not isinstance(reply.get("state"), str)
                or type(reply.get("fault_bits")) is not int):
            raise ValueError("runtime status is incomplete")
        print(json.dumps(reply, ensure_ascii=False, indent=2))
        if "error" in reply:
            raise SystemExit(2)
'''.replace("SOCKET_PATH", repr(runtime_socket)).replace("TIMEOUT", repr(timeout))


def read_status(peer):
    if not peer or peer.startswith("-"):
        raise ValueError("invalid SSH target")
    result = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", peer,
         "/usr/bin/python3 -"], input=remote_source(), text=True,
        capture_output=True, check=True, timeout=8)
    return json.loads(result.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--peer", required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(read_status(args.peer), ensure_ascii=False, indent=2))
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        parser.exit(1, "Status read failed: " + str(exc) + "\n")


if __name__ == "__main__":
    main()
