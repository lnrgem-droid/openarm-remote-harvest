#!/usr/bin/env python3
"""Detached remote supervisor: always retain log and final exit status."""
import json
from pathlib import Path
import subprocess
import time

bundle = Path(__file__).resolve().parent
try:
    result = subprocess.run(["/usr/bin/python3", "-u", str(bundle / "install_on_jetson.py")])
    code = result.returncode
except Exception as exc:
    print(str(exc), flush=True)
    code = 1
temp = bundle / "result.json.tmp"
temp.write_text(json.dumps({"returncode": code, "finished_unix": time.time()}))
temp.replace(bundle / "result.json")
