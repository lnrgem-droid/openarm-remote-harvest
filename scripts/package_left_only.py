#!/usr/bin/env python3
"""Create a portable source snapshot; no robot/network access."""
import argparse
from pathlib import Path
import shutil
import hashlib
import tarfile

ROOT=Path(__file__).resolve().parents[1]
NAME='openarm-left-only-20261113'
PACKAGES=['openarm_can','openarm_description','openarm_gravity_pd_control',
          'remote_teleop_protocol','remote_teleop_follower_safety','remote_teleop_runtime']
SCRIPTS=['left_only_device.py','left_only_env.sh','left_only_session.py','run_left_only_local.sh',
         'install_left_only.py','verify_left_release.py','verify_startup_alignment.py','startup_alignment_policy.json','left_startup_alignment_policy.json']


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    out=a.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    stage=out/NAME
    if stage.exists(): raise RuntimeError('目标目录已存在，拒绝覆盖：'+str(stage))
    for name in PACKAGES:
        shutil.copytree(ROOT/'ros2_robot/src'/name, stage/'ros2_robot/src'/name,
                        ignore=shutil.ignore_patterns('.git','__pycache__','.pytest_cache','*.pyc'))
    (stage/'scripts').mkdir()
    for name in SCRIPTS: shutil.copy2(ROOT/'scripts'/name,stage/'scripts'/name)
    (stage/'tests').mkdir()
    for name in ['test_left_only_deployment.py','left_only_alignment_fixture.py']:
        shutil.copy2(ROOT/'tests'/name,stage/'tests'/name)
    shutil.copy2(ROOT/'docs/LEFT_ONLY_20261113.md',stage/'README.md')
    files=sorted(f for f in stage.rglob('*') if f.is_file())
    def digest(f): return hashlib.sha256(f.read_bytes()).hexdigest()
    (stage/'release.sha256').write_text(''.join(f'{digest(f)}  {f.relative_to(stage)}\n' for f in files))
    archive=out/(NAME+'.tar.gz')
    with tarfile.open(archive,'w:gz') as tar: tar.add(stage,arcname=NAME)
    (out/(archive.name+'.sha256')).write_text(digest(archive)+'  '+archive.name+'\n')
    print(archive)

if __name__=='__main__': main()
