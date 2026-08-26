"""WebSocket 服务端。

在独立线程里跑 asyncio 事件循环，与 ROS 解耦：
  - 收到帧 -> 调用 on_message(frame, client_id)，可返回一个 dict 作为应答
  - broadcast(frame) 线程安全，供 ROS 定时器线程推送状态/事件
"""

import asyncio
import collections
import threading
from http import HTTPStatus

import websockets

from . import protocol


class WsServer:
    def __init__(self, host: str, port: int, on_message, logger=None):
        self._host = host
        self._port = port
        self._on_message = on_message
        self._log = logger
        self._loop: asyncio.AbstractEventLoop | None = None
        self._clients: set = set()
        self._latest_frames: dict = {}
        self._pending_frames: dict = {}
        self._send_events: dict = {}
        self._sender_tasks: dict = {}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._started = threading.Event()
        self._startup_error: BaseException | None = None

    # ---- 生命周期 ----
    def start(self, timeout: float = 3.0):
        self._thread = threading.Thread(target=self._run, name="ws_server", daemon=True)
        self._thread.start()
        self._started.wait(timeout)
        if self._startup_error is not None:
            raise RuntimeError(
                f"WebSocket 端口绑定失败 ws://{self._host}:{self._port}: "
                f"{self._startup_error}"
            ) from self._startup_error
        if self._loop is None:
            raise RuntimeError("WebSocket 服务启动超时")

    def stop(self):
        self._stop.set()
        if self._loop is not None and not self._loop.is_closed():
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except RuntimeError:
                pass

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
            self._started.set()
            self._loop.run_forever()
        except BaseException as e:
            self._startup_error = e
            self._started.set()
            self._warn(f"WebSocket 服务启动失败: {e}")
        finally:
            self._loop.close()

    async def _serve(self):
        await websockets.serve(
            self._handler,
            self._host,
            self._port,
            process_request=self._process_request,
        )
        self._info(f"WebSocket 服务已启动 ws://{self._host}:{self._port}")

    @staticmethod
    def _process_request(connection, request):
        """普通 HTTP 探测返回健康状态，WebSocket Upgrade 继续握手。"""
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None
        return connection.respond(HTTPStatus.OK, "robot_bridge websocket is running\n")

    # ---- 连接处理 ----
    async def _handler(self, ws):
        self._clients.add(ws)
        self._latest_frames[ws] = None
        self._pending_frames[ws] = collections.deque()
        self._send_events[ws] = asyncio.Event()
        self._sender_tasks[ws] = asyncio.create_task(self._send_loop(ws))
        peer = getattr(ws, "remote_address", None)
        self._info(f"客户端接入 {peer}，当前 {len(self._clients)} 个")
        try:
            async for raw in ws:
                await self._dispatch(ws, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._clients.discard(ws)
            self._latest_frames.pop(ws, None)
            self._pending_frames.pop(ws, None)
            self._send_events.pop(ws, None)
            sender_task = self._sender_tasks.pop(ws, None)
            if sender_task is not None:
                sender_task.cancel()
            self._info(f"客户端断开 {peer}，剩余 {len(self._clients)} 个")

    async def _dispatch(self, ws, raw):
        try:
            frame = protocol.decode(raw)
        except ValueError as e:
            await ws.send(protocol.encode(protocol.make_error(str(e), code="BAD_FRAME")))
            return
        try:
            reply = self._on_message(frame, id(ws))
        except Exception as e:  # noqa: BLE001 桥接回调异常不应断开连接
            self._warn(f"处理帧异常: {e}")
            reply = protocol.make_error(str(e), code="HANDLER_ERROR", req=frame)
        if reply is not None:
            await ws.send(protocol.encode(reply))

    # ---- 推送 ----
    def broadcast(self, frame: dict):
        """线程安全广播给所有客户端。"""
        if self._loop is None or self._loop.is_closed() or not self._clients:
            return
        data = protocol.encode(frame)
        latest_only = frame.get("type") == protocol.TYPE_STATE
        try:
            self._loop.call_soon_threadsafe(self._broadcast_soon, data, latest_only)
        except RuntimeError:
            pass

    def _broadcast_soon(self, data: str, latest_only: bool):
        for ws in list(self._clients):
            event = self._send_events.get(ws)
            if event is None:
                continue
            if latest_only:
                # 高频状态只保留最新帧。发送较慢时，新状态覆盖尚未发送的旧状态。
                self._latest_frames[ws] = data
            else:
                # 低频事件和错误不能被状态帧覆盖，交给同一个发送任务顺序发送。
                self._pending_frames[ws].append(data)
            event.set()

    async def _send_loop(self, ws):
        """单客户端单发送任务：慢客户端只丢旧帧，不积压发送任务。"""
        event = self._send_events[ws]
        while True:
            await event.wait()
            event.clear()

            pending = self._pending_frames.get(ws)
            if pending:
                data = pending.popleft()
                if pending or self._latest_frames.get(ws) is not None:
                    event.set()
            else:
                data = self._latest_frames.get(ws)
                self._latest_frames[ws] = None
            if data is None:
                continue

            try:
                await ws.send(data)
            except websockets.ConnectionClosed:
                self._clients.discard(ws)
                return

    # ---- 日志 ----
    def _info(self, msg):
        if self._log:
            try:
                self._log.info(msg)
            except Exception:
                print(f"[robot_bridge][ws] {msg}", flush=True)

    def _warn(self, msg):
        if self._log:
            try:
                self._log.warning(msg)
            except Exception:
                print(f"[robot_bridge][ws][WARN] {msg}", flush=True)
