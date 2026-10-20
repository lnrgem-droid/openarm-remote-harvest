#!/usr/bin/env python3
"""Offline, idle-only updater for the existing nvidia Jetson installation."""
import ast
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import time
import uuid

HOME = Path("/home/nvidia")
CONTROL = HOME / "dev/openarm-remote-harvest"
RGBD = HOME / "dev/openarm-rgbd-preview"
ROS = CONTROL / "ros2_robot"
STATE = HOME / "openarm-offline-updates"
PACKAGES = ("remote_teleop_protocol", "remote_teleop_runtime", "openarm_gravity_pd_control")
PREFIXES = ("install", "install_bimanual", "install_gripper_candidate",
            "install_tracking_candidate", "install_gripper_spring_candidate")
ACTIVE_NAMES = ("openarm_gravity_pd_node", "remote-teleop-leader", "remote-teleop-follower",
                "jetson_record_manager.py", "jetson_orbbec_rgbd_service.py",
                "record_openarm_dataset.py", "record_openarm_fast_staging.py",
                "record_jetson_rgbd_dataset.sh", "lerobot.scripts.lerobot_record",
                "record_rgbd", "bridge_node", "robot_bridge", "remote_teleop_runtime",
                "mushroom_collection_console.py", "run_bimanual_remote_feedback.sh")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def exists(path):
    return Path(path).exists() or Path(path).is_symlink()


def remove(path):
    path = Path(path)
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.exists():
        shutil.rmtree(path)


def active_processes(proc=Path("/proc")):
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError):
            continue
        # Match argv elements, not a shell's quoted inspection command.
        names = [os.path.basename(a.decode(errors="replace")) for a in args if a]
        if any(any(name == token or name.startswith(token + "-")
                   for token in ACTIVE_NAMES) for name in names):
            found.append({"pid": int(entry.name), "argv": [a.decode(errors="replace") for a in args if a]})
    return found


def require_idle():
    running = active_processes()
    if running:
        raise RuntimeError("检测到控制/相机/录制进程，未进行更新。先安全停止遥操并退出采集；"
                           "可在 Jetson 正常重启后、打开采集界面前再更新。\n" +
                           json.dumps(running, ensure_ascii=False))
    if Path("/tmp/openarm-rgbd-recording.active").exists():
        raise RuntimeError("存在录制活动标记，拒绝更新。请先确认录制已完整保存。")


def safe_extract(archive, dest, prefix):
    dest = Path(dest)
    with tarfile.open(archive) as stream:
        members = stream.getmembers()
        if sum(m.size for m in members) > 512 * 1024 * 1024:
            raise RuntimeError("压缩包展开尺寸异常")
        for member in members:
            parts = Path(member.name).parts
            if (not parts or parts[0] != prefix or ".." in parts or
                    Path(member.name).is_absolute() or not (member.isfile() or member.isdir())):
                raise RuntimeError("压缩包包含不安全路径/链接：" + member.name)
        stream.extractall(dest, members=members)


def ros_command(command, extra=None):
    sources = [Path("/opt/ros/humble/setup.bash"),
               ROS / "install/setup.bash", ROS / "install_bimanual/setup.bash",
               ROS / "install_gripper_candidate/setup.bash",
               ROS / "install_gripper_spring_candidate/local_setup.bash"]
    if extra:
        sources.append(Path(extra) / "local_setup.bash")
    script = "set -eo pipefail\n" + "\n".join("source " + shlex.quote(str(p)) for p in sources)
    script += "\n" + command
    return ["bash", "-c", script]


def run_ros(command, extra=None, **kwargs):
    env = {"HOME": str(HOME), "USER": "nvidia", "LOGNAME": "nvidia",
           "PATH": "/usr/bin:/bin:/usr/sbin:/sbin", "LANG": "C.UTF-8",
           "PYTHONDONTWRITEBYTECODE": "1", "CMAKE_BUILD_PARALLEL_LEVEL": "2"}
    return subprocess.run(ros_command(command, extra), env=env, check=True, **kwargs)


def preflight():
    if platform.machine() != "aarch64" or Path.home() != HOME:
        raise RuntimeError("仅允许在原 Jetson 的 nvidia 用户下安装；不能在笔记本/旧主机执行。")
    require_idle()
    for path in [CONTROL, RGBD, Path("/opt/ros/humble/setup.bash"),
                 ROS / "install/setup.bash", ROS / "install_bimanual/setup.bash",
                 ROS / "install_gripper_candidate/setup.bash",
                 ROS / "install_gripper_spring_candidate/local_setup.bash",
                 RGBD / "ros2_robot/install/setup.bash",
                 HOME / "miniconda3/envs/lerobot/bin/python"]:
        if not path.exists():
            raise RuntimeError("原安装不完整，缺少：" + str(path))
    for command in ("colcon", "cmake", "make", "g++"):
        if not Path("/usr/bin", command).exists():
            raise RuntimeError("缺少离线编译依赖：" + command)
    if shutil.disk_usage(HOME).free < 4 * 1024**3:
        raise RuntimeError("Jetson 可用空间不足 4 GB，请先腾出空间。")
    run_ros("/usr/bin/python3 -c " + shlex.quote(
        "import rclpy,yaml; from ament_index_python.packages import get_package_prefix; "
        "[get_package_prefix(x) for x in ['openarm_can','openarm_description',"
        "'remote_teleop_follower_safety']]"))
    subprocess.run([str(HOME / "miniconda3/envs/lerobot/bin/python"), "-c",
                    "import numpy,zmq,cv2,pyarrow,pandas,yaml,pyorbbecsdk"],
                   check=True, timeout=40)


def overlay_files(source, target):
    """Overlay payload on a private copy; never follow an old destination symlink."""
    for path in Path(source).rglob("*"):
        rel = path.relative_to(source)
        if "__pycache__" in rel.parts or path.suffix == ".pyc":
            continue
        dest = Path(target) / rel
        if path.is_dir():
            if dest.is_symlink():
                remove(dest)
            dest.mkdir(parents=True, exist_ok=True)
        elif path.is_file():
            dest.parent.mkdir(parents=True, exist_ok=True)
            if dest.is_symlink():
                dest.unlink()
            shutil.copy2(path, dest)


def prepare_sources(extracted, job):
    src = job / "src"
    src.mkdir()
    preserved = []
    for package in PACKAGES:
        source = extracted / "remote-control/ros2_robot/src" / package
        target = src / package
        shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        # Preserve the settings actually used by the existing follower. New
        # control behavior is implemented in code; no retuning is done here.
        installed = None
        for prefix in PREFIXES:
            candidate = ROS / prefix / package / "share" / package / "config"
            if candidate.is_dir():
                installed = candidate
        if installed:
            for path in installed.glob("*.yaml"):
                dest = target / "config" / path.name
                dest.parent.mkdir(exist_ok=True)
                if not dest.exists() or digest(dest) != digest(path):
                    preserved.append(str(path))
                shutil.copy2(path, dest)
    write_json(job / "preserved-configs.json", preserved)
    return src


def verify_install(install, src, actual=False):
    # Imports/ldd only: never execute a controller, launch file or ROS service.
    code = """
import hashlib,json,pathlib,platform
from ament_index_python.packages import get_package_prefix
import remote_teleop_protocol,remote_teleop_runtime
from remote_teleop_runtime.collection_motion import CollectionMotion
from remote_teleop_runtime.follower_io import FollowerIOWorker
for name in ['remote_teleop_protocol','remote_teleop_runtime']:
 module=__import__(name)
 folder=pathlib.Path(module.__file__).parent
 source=pathlib.Path(SRC)/name/name
 for f in source.glob('*.py'):
  assert hashlib.sha256(f.read_bytes()).digest()==hashlib.sha256((folder/f.name).read_bytes()).digest(),str(f)
prefix=pathlib.Path(get_package_prefix('openarm_gravity_pd_control'))
binary=prefix/'lib/openarm_gravity_pd_control/openarm_gravity_pd_node'
expected=pathlib.Path(INSTALL)/'openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node'
assert binary.resolve()==expected.resolve(),str(binary)
expected_machine={'aarch64':b'\\xb7\\x00','x86_64':b'\\x3e\\x00'}[platform.machine()]
assert binary.read_bytes()[18:20]==expected_machine,'wrong native ELF architecture'
print('实际模块及本机架构控制程序检查通过：'+platform.machine())
""".replace("SRC", repr(str(src))).replace("INSTALL", repr(str(install)))
    extra = None if actual else install
    run_ros("/usr/bin/python3 -c " + shlex.quote(code), extra)
    binary = install / "openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
    result = run_ros("ldd " + shlex.quote(str(binary)), extra, capture_output=True, text=True)
    if "not found" in result.stdout:
        raise RuntimeError("控制程序缺少动态库：\n" + result.stdout)


class Transaction:
    def __init__(self, job):
        self.job = Path(job)
        self.journal = self.job / "transaction.json"
        self.data = {"status": "prepared", "entries": []}

    def load(self):
        self.data = json.loads(self.journal.read_text())
        return self

    def save(self):
        write_json(self.journal, self.data)

    def replace(self, dest, candidate):
        dest, candidate = Path(dest), Path(candidate)
        backup = self.job / "backup" / str(len(self.data["entries"]))
        backup.parent.mkdir(exist_ok=True)
        item = {"dest": str(dest), "candidate": str(candidate), "backup": str(backup),
                "had_original": exists(dest)}
        self.data["entries"].append(item)
        self.data["status"] = "activating"
        self.save()  # Write ahead: a power failure between renames is recoverable.
        if item["had_original"]:
            os.replace(dest, backup)
        os.replace(candidate, dest)

    def rollback(self):
        for item in reversed(self.data["entries"]):
            dest, backup = Path(item["dest"]), Path(item["backup"])
            if exists(backup):
                remove(dest)
                os.replace(backup, dest)
            elif not item["had_original"] and not exists(item["candidate"]):
                remove(dest)
        self.data["status"] = "rolled_back"
        self.save()

    def commit(self):
        self.data["status"] = "complete"
        self.save()


def recover_incomplete():
    for journal in sorted(STATE.glob("job-*/transaction.json")):
        tx = Transaction(journal.parent).load()
        if tx.data["status"] == "activating":
            require_idle()
            print("恢复上次中断的更新，先还原旧版本：" + str(journal.parent), flush=True)
            tx.rollback()


def install(bundle):
    recover_incomplete()
    preflight()
    manifest = json.loads((bundle / "manifest.json").read_text())
    for name in ("remote-control", "rgbd-capture"):
        if digest(bundle / (name + ".tar.gz")) != manifest[name]["sha256"]:
            raise RuntimeError("更新包校验失败：" + name)
    job = STATE / ("job-" + time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8])
    job.mkdir()
    write_json(job / "manifest.json", manifest)
    tx = Transaction(job)
    try:
        print("1/4 展开已校验的离线包，保留现有配置；工作目录：" + str(job), flush=True)
        extracted = job / "payload"
        for name in ("remote-control", "rgbd-capture"):
            safe_extract(bundle / (name + ".tar.gz"), extracted, name)
        src = prepare_sources(extracted, job)
        native = job / "install"
        print("2/4 在 Jetson 本机编译；可能需要数分钟，请不要断电。", flush=True)
        command = shlex.join(["colcon", "--log-base", str(job / "log"), "build",
                              "--base-paths", str(src), "--build-base", str(job / "build"),
                              "--install-base", str(native), "--executor", "sequential",
                              "--packages-select", *PACKAGES, "--cmake-args",
                              "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"])
        run_ros(command, cwd=job)
        verify_install(native, src)
        candidates = job / "candidates"
        candidates.mkdir()
        changes = []
        for package in PACKAGES:
            dest = ROS / "src" / package
            candidate = candidates / ("src-" + package)
            shutil.copytree(src / package, candidate)
            changes.append((dest, candidate))
        # Update existing prefixes so every current launcher/import order sees
        # the same tested modules. Original package directories remain backed up.
        for prefix in PREFIXES:
            for package in PACKAGES:
                dest = ROS / prefix / package
                if exists(dest):
                    candidate = candidates / (prefix + "-" + package)
                    candidate.symlink_to(native / package, target_is_directory=True)
                    changes.append((dest, candidate))
        dest = RGBD / "scripts"
        candidate = candidates / "rgbd-scripts"
        shutil.copytree(dest, candidate, symlinks=True)
        overlay_files(extracted / "rgbd-capture/scripts", candidate)
        for path in candidate.glob("*.py"):
            ast.parse(path.read_text(), filename=str(path))
        for path in candidate.glob("*.sh"):
            subprocess.run(["bash", "-n", str(path)], check=True)
        changes.append((dest, candidate))
        # Configurations, datasets, calibration, SSH, IP and camera serials are
        # deliberately outside the transaction. Source snapshots contain more
        # files than need installation on a follower.
        require_idle()
        print("3/4 编译已通过，备份并切换程序。", flush=True)
        for dest, candidate in changes:
            tx.replace(dest, candidate)
        print("4/4 核对实际加载路径和程序完整性。", flush=True)
        verify_install(native, src, actual=True)
        for path in (extracted / "rgbd-capture/scripts").glob("*.py"):
            if digest(path) != digest(RGBD / "scripts" / path.name):
                raise RuntimeError("采集程序校验失败：" + path.name)
        receipt = {"status": "installed", "job": str(job), "manifest": manifest,
                   "native_binary_sha256": digest(native / "openarm_gravity_pd_control/lib/"
                                                "openarm_gravity_pd_control/openarm_gravity_pd_node"),
                   "hardware_tested": False, "completed_unix": time.time()}
        tx.commit()
        write_json(job / "receipt.json", receipt)
        write_json(STATE / "latest-success.json", receipt)
        print("\n软件更新完成：实际运行模块和本机编译结果已核验。", flush=True)
        print("未启动遥操、相机或归位；请打开采集控制台，检查三路画面并做短录制验证。", flush=True)
        print("备份与日志：" + str(job), flush=True)
    except BaseException:
        if tx.data["status"] == "activating":
            print("更新未完成，正在还原原程序。", flush=True)
            tx.rollback()
        raise


def main():
    bundle = Path(__file__).resolve().parent
    if platform.machine() != "aarch64" or Path.home() != HOME:
        raise RuntimeError("仅允许在原 Jetson 的 nvidia 用户下安装。")
    STATE.mkdir(exist_ok=True)
    with (STATE / "update.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("已有更新任务运行，请等待它完成。")
        signal.signal(signal.SIGTERM, lambda *_: sys.exit("更新被终止"))
        install(bundle)


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print("\n更新未完成：" + str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
