#!/usr/bin/env python3
"""Build an isolated native overlay and optionally install inactive user units."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
from verify_left_release import ROOT, digest, verify


def unit_text(root, role, bus, ip):
    # systemd syntax differs from shell quoting; reject unsupported path chars.
    if any(c in str(root) for c in '\n\r"%\\'): raise ValueError('unsupported installation path')
    return f'''[Unit]
Description=OpenArm physical LEFT only ({role})
[Service]
Type=exec
WorkingDirectory={root}
ExecStart=/bin/bash "{root}/scripts/run_left_only_local.sh" {role} {bus} {ip}
Restart=no
TimeoutStopSec=15
KillMode=control-group
Environment=PYTHONUNBUFFERED=1
'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role', choices=['leader', 'follower'], required=True)
    p.add_argument('--underlay', type=Path, default=Path.home()/'dev/openarm-remote-harvest/ros2_robot/install')
    p.add_argument('--leader-bus', choices=['can1'], default='can1')
    p.add_argument('--follower-bus', choices=['can1','can2'], default='can2')
    p.add_argument('--peer', default='openarm-jetson', help='SSH alias from laptop')
    p.add_argument('--peer-ip', default='192.168.50.2')
    p.add_argument('--peer-root', default='/home/nvidia/openarm-left-only-20261113')
    p.add_argument('--build-only', action='store_true', help='compile/validate without changing user units')
    a = p.parse_args()
    import ipaddress
    ipaddress.ip_address(a.peer_ip)
    if a.peer.startswith('-'): p.error('invalid SSH peer')
    if not Path(a.peer_root).is_absolute(): p.error('--peer-root must be absolute')
    underlay = a.underlay.resolve()
    if not (underlay/'setup.bash').is_file(): p.error('原 ROS 工作空间不存在，请指定 --underlay')
    verify(built=False)
    # Rebuilding loaded code could mix versions; refuse even with build-only.
    from left_only_device import controller_owners
    for owner in controller_owners():
        try: cmd = Path(f'/proc/{owner["pid"]}/cmdline').read_bytes()
        except FileNotFoundError: continue
        if str(ROOT).encode() in cmd:
            raise RuntimeError('本更新包的控制器正在运行，禁止覆盖编译')
    ws = ROOT/'ros2_robot'
    packages = ['openarm_description', 'openarm_gravity_pd_control', 'remote_teleop_protocol',
                'remote_teleop_follower_safety', 'remote_teleop_runtime']
    base = 'colcon --log-base log_left_only build --build-base build_left_only --install-base install_left_only --merge-install --executor sequential '
    script = 'set -eo pipefail\nsource /opt/ros/humble/setup.bash\nsource '+shlex.quote(str(underlay/'setup.bash'))+'\n'
    script += 'export CMAKE_BUILD_PARALLEL_LEVEL=2\n'
    script += base+'--packages-select openarm_can --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release\n'
    script += 'source install_left_only/local_setup.bash\n'
    script += base+'--packages-select '+shlex.join(packages)+' --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release\n'
    subprocess.run(['bash','-c',script], cwd=ws, check=True)
    (ROOT/'scripts/left_only_paths.sh').write_text('LEFT_UNDERLAY='+shlex.quote(str(underlay))+'\n')
    c = dict(role=a.role, leader_bus=a.leader_bus, follower_bus=a.follower_bus,
             peer=a.peer, peer_ip=a.peer_ip, peer_root=a.peer_root)
    (ROOT/'left_only_config.json').write_text(json.dumps(c, indent=2)+'\n')
    overlay = ws/'install_left_only'
    files = sorted(f for f in overlay.rglob('*') if f.is_file())
    if not files: raise RuntimeError('empty build')
    (ROOT/'installed.sha256').write_text(''.join(f'{digest(f)}  {f.relative_to(ROOT)}\n' for f in files))
    (ROOT/'built_from.sha256').write_text(digest(ROOT/'release.sha256')+'\n')
    verify()
    # Imports and xacro are safe: no node/controller/CAN is constructed.
    smoke = ('set -e; source '+shlex.quote(str(ROOT/'scripts/left_only_env.sh'))+
             '; /usr/bin/python3 -c '+shlex.quote('from remote_teleop_runtime.left_only_launch import description; '
             'from remote_teleop_runtime.leader import LeaderGateway; '
             'from remote_teleop_runtime.follower import FollowerGateway; '
             'assert description("leader") and description("follower"); print("left-only imports OK")'))
    subprocess.run(['bash','-c',smoke], check=True)
    if not a.build_only:
        unit = 'openarm-left-'+a.role+'.service'
        status = subprocess.run(['systemctl','--user','show',unit,'-p','MainPID','-p','ActiveState'], capture_output=True, text=True, check=True).stdout
        fields = dict(line.split('=',1) for line in status.splitlines() if '=' in line)
        if fields.get('MainPID') != '0' or fields.get('ActiveState') not in ('inactive','failed'):
            raise RuntimeError('单左臂服务未停止，不覆盖服务文件')
        directory = Path.home()/'.config/systemd/user'; directory.mkdir(parents=True,exist_ok=True)
        (directory/unit).write_text(unit_text(ROOT, a.role, a.leader_bus if a.role=='leader' else a.follower_bus, a.peer_ip))
        subprocess.run(['systemctl','--user','daemon-reload'],check=True)
        subprocess.run(['systemd-analyze','--user','verify',str(directory/unit)],check=True)
        loaded = subprocess.run(['systemctl','--user','show',unit,'-p','LoadState','--value'],
                                text=True,capture_output=True,check=True).stdout.strip()
        if loaded != 'loaded': raise RuntimeError('单左臂服务未正确加载：'+loaded)
    print('安装/编译完成。没有启动、使能电机或设置开机启动。')

if __name__ == '__main__': main()
