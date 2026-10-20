#!/usr/bin/env python3
"""Read-only field connection/source check. Never enables CAN or motors."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def run(args, **kwargs):
    return subprocess.run(args, text=True, capture_output=True, timeout=25, **kwargs)


def main():
    print("OpenArm 现场只读检查：不使能、不归位、不启动相机或遥操。", flush=True)
    for args in (["ip", "-brief", "address", "show", "enp59s0"],
                 ["ip", "-brief", "link", "show", "can0"],
                 ["ip", "-brief", "link", "show", "can1"]):
        result = run(args)
        print((result.stdout + result.stderr).strip())
    expected = json.loads((HERE / "expected-jetson-sources.json").read_text())
    code = r'''
import json,hashlib,pathlib,platform,subprocess,shutil
expected=EXPECTED
missing=[];different=[]
for name,digest in expected.items():
 p=pathlib.Path(name)
 if not p.is_file():missing.append(name)
 elif hashlib.sha256(p.read_bytes()).hexdigest()!=digest:different.append(name)
root=pathlib.Path('/home/nvidia/dev/openarm-remote-harvest/ros2_robot')
required=[root/'install/setup.bash',root/'install_bimanual/setup.bash',
 root/'install_gripper_candidate/setup.bash',root/'install_gripper_spring_candidate/local_setup.bash',
 root/'install_gripper_spring_candidate/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node',
 pathlib.Path('/home/nvidia/miniconda3/envs/lerobot/bin/python'),
 pathlib.Path('/home/nvidia/openarm_robot/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan')]
interfaces={i:subprocess.run(['ip','-brief','link','show',i],capture_output=True,text=True).stdout.strip() for i in ['can1','can2']}
print(json.dumps(dict(host=platform.node(),arch=platform.machine(),source_missing=missing,source_different=different,
 missing_dependencies=[str(p) for p in required if not p.exists()],interfaces=interfaces,
 free_gb=round(shutil.disk_usage('/home/nvidia').free/1024**3,1))))
'''.replace("EXPECTED", repr(expected))
    result = run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                  "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
                  "openarm-jetson", "/usr/bin/python3", "-"], input=code)
    if result.returncode:
        print("\n未通过：Jetson SSH 尚未连通。检查 Jetson 电源、直连网线和 192.168.50.2。")
        print(result.stderr.strip())
        print("这不是安装成功，也不是机械臂故障诊断；当前没有发送控制命令。")
        return 2
    report = json.loads(result.stdout)
    (HERE / "last-field-check.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\n已连接 {report['host']}（{report['arch']}），可用空间 {report['free_gb']} GB")
    for name, state in report["interfaces"].items():
        print(f"{name}: {state or '未发现接口'}")
    for key, label in (("source_missing", "缺少源码"), ("source_different", "源码版本不同"),
                       ("missing_dependencies", "缺少依赖/安装文件")):
        print(f"{label}：{len(report[key])} 项")
        for name in report[key][:12]:
            print("  " + name)
    blocked = any(report[k] for k in ("source_missing", "source_different", "missing_dependencies"))
    blocked |= any(not value for value in report["interfaces"].values())
    print("\n仍须验证 Jetson 实际加载的 ROS 模块及本机编译结果、三相机取流、短录制完整性。")
    print("源码一致也不代表以上验收已完成；本检查不会自动安装或启动运动。")
    print("离线更新包：/home/openarm/openarm-transfer/jetson-update-20261020")
    return 1 if blocked else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (subprocess.TimeoutExpired, OSError, ValueError) as exc:
        print("检查未完成：" + str(exc))
        sys.exit(2)
