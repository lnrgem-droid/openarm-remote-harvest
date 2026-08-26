"""Web entry point for the ESO-Robot browser control panel.

Run from the workspace root:
    python3 -m uvicorn web.server:app --host 0.0.0.0 --port 8000

The browser connects to /ws/robot. This app proxies that socket to the existing
robot_bridge WebSocket server, usually ws://127.0.0.1:9000 on the robot.
"""

import asyncio
import os
from pathlib import Path

import websockets
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


WEB_DIR = Path(__file__).resolve().parent
BRIDGE_URL = os.getenv("ROBOT_BRIDGE_WS_URL", "ws://127.0.0.1:9000")

app = FastAPI(title="ESO-Robot Control")


@app.get("/")
async def index():
    return FileResponse(WEB_DIR / "index.html")


@app.get("/api/config")
async def config():
    return {"bridge_ws_url": "/ws/robot", "upstream_bridge_ws_url": BRIDGE_URL}


@app.websocket("/ws/robot")
async def robot_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        async with websockets.connect(BRIDGE_URL) as upstream:
            browser_to_robot = asyncio.create_task(_browser_to_robot(websocket, upstream))
            robot_to_browser = asyncio.create_task(_robot_to_browser(websocket, upstream))
            done, pending = await asyncio.wait(
                {browser_to_robot, robot_to_browser},
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in pending:
                task.cancel()
            for task in done:
                task.result()
    except WebSocketDisconnect:
        return
    except Exception as exc:  # noqa: BLE001
        try:
            await websocket.send_json({
                "type": "error",
                "code": "BRIDGE_PROXY_ERROR",
                "message": f"无法连接 robot_bridge 上游 {BRIDGE_URL}: {exc}",
            })
        except Exception:
            pass


async def _browser_to_robot(websocket: WebSocket, upstream):
    while True:
        await upstream.send(await websocket.receive_text())


async def _robot_to_browser(websocket: WebSocket, upstream):
    async for message in upstream:
        await websocket.send_text(message)


app.mount("/", StaticFiles(directory=WEB_DIR, html=True), name="web")
