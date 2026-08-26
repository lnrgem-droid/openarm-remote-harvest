# ESO-Robot Web 控制台

精简版浏览器控制台，用于通过 `robot_bridge WebSocket` 控制机器人。

## 安装依赖

在工作空间根目录执行：

```bash
python3 -m pip install -r web/requirements.txt
```

## 启动

默认连接本机 `robot_bridge`：`ws://127.0.0.1:9000`。

先启动 ROS 端桥接节点：

```bash
source install/setup.bash
ros2 launch robot_bridge bridge.launch.py
```

再启动 Web 页面服务：

```bash
python3 -m uvicorn web.server:app --host 0.0.0.0 --port 8000
```

浏览器访问：

```text
http://127.0.0.1:8000
```

局域网手机访问时，把 `127.0.0.1` 换成运行该服务电脑的 IP。

## 指定 robot_bridge 地址

如果 `robot_bridge` 不在本机或端口不同：

```bash
ROBOT_BRIDGE_WS_URL=ws://<robot-ip>:9000 python3 -m uvicorn web.server:app --host 0.0.0.0 --port 8000
```

页面内也可以点击顶部连接状态，手动修改 WebSocket 地址。
