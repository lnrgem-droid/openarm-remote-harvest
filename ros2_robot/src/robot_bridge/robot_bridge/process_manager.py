"""ROS2 launch 进程管理。

只负责按照 interface_list.md 里的启动示例拉起底层 ROS2 节点。
不处理 WebSocket，不处理具体控制命令。
"""

import subprocess
import shutil
import os
import signal
import time
from pathlib import Path


class ProcessManager:
    def __init__(self, config: dict, logger=None):
        self._logger = logger
        launches = config["launches"]
        self._arm_launches = launches["arm_modes"]        # mode -> [[name, cmd], ...]
        self._component_launches = launches["components"]  # component -> cmd
        self._default_components = config["startup"]["default_components"]
        self._log_dir = Path(config["startup"]["log_dir"])
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._processes: dict[str, subprocess.Popen] = {}

    def startup(
        self,
        arm_mode: str,
        components: list[str] | None = None,
        show_terminal: bool = False,
    ) -> dict:
        """按组件启动底层 ROS2 launch。components 缺省用 config 里的 default_components。"""
        if components is None:
            components = self._default_components
        result = {}
        for component in components:
            if component == "arms":
                result["arms"] = self._start_arm(arm_mode, show_terminal)
            elif component in self._component_launches:
                result[component] = self._start(
                    component, self._component_launches[component], show_terminal)
            else:
                self._info(f"未知组件，跳过: {component}")
        return result

    def shutdown(self, timeout: float = 5.0) -> dict:
        """停止本管理器启动的 launch 进程，并返回停止结果。"""
        result = {}
        for name, proc in list(self._processes.items()):
            result[name] = self._stop_process(name, proc, timeout)
        self._processes.clear()
        return result

    def _start_arm(self, arm_mode: str, show_terminal: bool) -> dict:
        """启动某个双臂模式所需的全部 launch（cartesian 模式会有多条）。"""
        procs = self._arm_launches.get(arm_mode)
        if procs is None:
            raise ValueError(f"不支持的双臂模式: {arm_mode}")
        result = {}
        for name, cmd in procs:
            result[name] = self._start(name, cmd, show_terminal)
        return result

    def _start(self, name: str, cmd: list[str], show_terminal: bool) -> dict:
        proc = self._processes.get(name)
        if proc is not None and proc.poll() is None:
            return {"status": "already_running", "pid": proc.pid, "cmd": cmd}

        log_path = self._log_dir / f"{name}.log"
        if show_terminal:
            terminal_cmd = self._terminal_cmd(name, cmd)
            if terminal_cmd is not None:
                proc = subprocess.Popen(terminal_cmd, start_new_session=True)
                self._processes[name] = proc
                self._info(
                    f"终端启动 launch: {name}, pid={proc.pid}, cmd={' '.join(cmd)}")
                return {
                    "status": "started_terminal",
                    "pid": proc.pid,
                    "cmd": cmd,
                    "terminal_cmd": terminal_cmd,
                }
            self._info(f"未找到可用图形终端，{name} 回退到后台日志: {log_path}")

        log_file = log_path.open("ab")
        proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._processes[name] = proc
        self._info(f"启动 launch: {name}, pid={proc.pid}, cmd={' '.join(cmd)}, log={log_path}")
        return {"status": "started", "pid": proc.pid, "cmd": cmd, "log": str(log_path)}

    def _stop_process(self, name: str, proc: subprocess.Popen, timeout: float) -> dict:
        # start_new_session=True 保证 pgid == pid。ros2 launch 父进程可能先退出，
        # 所以不能只看 proc.poll()，必须确认整个进程组都已消失。
        if not self._process_group_exists(proc.pid):
            return {
                "status": "already_exited",
                "pid": proc.pid,
                "returncode": proc.poll(),
            }

        self._info(f"停止 launch 进程组: {name}, pgid={proc.pid}")
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return {"status": "not_found", "pid": proc.pid}
        except Exception as e:  # noqa: BLE001
            self._info(f"停止 {name} 失败: {e}")
            return {"status": "error", "pid": proc.pid, "error": str(e)}

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            proc.poll()
            if not self._process_group_exists(proc.pid):
                return {
                    "status": "terminated",
                    "pid": proc.pid,
                    "returncode": proc.returncode,
                }
            time.sleep(0.1)

        self._info(f"{name} 进程组未在 {timeout:.1f}s 内退出，强制结束")
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        kill_deadline = time.monotonic() + 1.0
        while time.monotonic() < kill_deadline:
            proc.poll()
            if not self._process_group_exists(proc.pid):
                return {"status": "killed", "pid": proc.pid}
            time.sleep(0.05)
        return {"status": "kill_timeout", "pid": proc.pid}

    @staticmethod
    def _process_group_exists(pgid: int) -> bool:
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _terminal_cmd(self, name: str, cmd: list[str]) -> list[str] | None:
        quoted_cmd = " ".join(shlex_quote(part) for part in cmd)
        shell_cmd = f"echo '[robot_bridge] {name}: {quoted_cmd}'; {quoted_cmd}; exec bash"

        if shutil.which("gnome-terminal"):
            return [
                "gnome-terminal",
                "--wait",
                "--title",
                f"robot_bridge:{name}",
                "--",
                "bash",
                "-lc",
                shell_cmd,
            ]
        if shutil.which("x-terminal-emulator"):
            return ["x-terminal-emulator", "-T", f"robot_bridge:{name}", "-e", f"bash -lc {shlex_quote(shell_cmd)}"]
        if shutil.which("xfce4-terminal"):
            return ["xfce4-terminal", "--title", f"robot_bridge:{name}", "--command", f"bash -lc {shlex_quote(shell_cmd)}"]
        if shutil.which("konsole"):
            return ["konsole", "--new-tab", "-p", f"tabtitle=robot_bridge:{name}", "-e", "bash", "-lc", shell_cmd]
        return None

    def _info(self, msg: str):
        if self._logger:
            try:
                self._logger.info(msg)
                return
            except Exception:
                pass
        print(f"[robot_bridge][process] {msg}", flush=True)


def shlex_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"
