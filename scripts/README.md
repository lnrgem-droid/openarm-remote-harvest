# ZMQ 相机推流

相机侧运行 `zmq_camera_publisher.py`，通过 ZMQ 推送画面给 LeRobot `ZMQCamera`。

```bash
pip install opencv-python pyzmq numpy
```

---

## 1. 查看摄像头

```bash
v4l2-ctl --list-devices

ls -l /dev/v4l/by-path/
```

每台相机会占两个节点（`video-index0` / `video-index1`），**采集一般用** `video-index0`。

示例输出对应关系：


| by-path                                     | /dev/video |
| ------------------------------------------- | ---------- |
| `pci-0000:00:14.0-usb-0:1:1.0-video-index0` | video2     |
| `pci-0000:00:14.0-usb-0:2:1.0-video-index0` | video0     |
| `pci-0000:00:14.0-usb-0:3:1.0-video-index0` | video4     |


查分辨率：

```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

---

## 2. 固定 USB 口（不要用 /dev/videoN）

重启后 `video0/2/4` 会变，用 **by-path** 绑定物理 USB 口：

```bash
ls -l /dev/v4l/by-path/ | grep video-index0
```

按实际画面确认左/右/头对应关系后，启动时用 `--camera NAME:by-path路径`。

---



## 3. 启动推流

**默认三路**（`/dev/video0/2/4`，仅本机调试）：

```bash
python3 /home/openarm/vla/scripts/zmq_camera_publisher.py
```

**推荐：by-path 固定 USB 口**（按你的机器示例，左/右/头命名需自行核对画面）：

```bash
python3 /home/openarm/vla/scripts/zmq_camera_publisher.py \
  --camera cam_left:/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0 \
  --camera cam_right:/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0 \
  --camera cam_head:/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0 \
  --port 5555 --fps 30 --width 640 --height 480
```

`Ctrl+C` 退出。日志每 2 秒打印发布 FPS 与延迟。

---



## 4. LeRobot 侧查看（Rerun，不采集）

LeRobot 环境需安装 `pyzmq`：

```bash
pip install pyzmq
# 或
pip install 'lerobot[pyzmq-dep]'
```

`server_address` 填推流机 IP，`camera_name` 与 `--camera` 名称一致，**必须带 width/height/fps**：

```bash
lerobot-teleoperate \
  --robot.type=openarm_bridge \
  --robot.ws_url=ws://192.168.0.66:9000 \
  --teleop.type=openarm_bridge_teleop \
  --teleop.ws_url=ws://192.168.0.66:9000 \
  --robot.control_authority=external \
  --robot.cameras='{
    cam_left:  {type: zmq, server_address: "192.168.0.66", port: 5555, camera_name: "cam_left",  width: 640, height: 480, fps: 30},
    cam_right: {type: zmq, server_address: "192.168.0.66", port: 5555, camera_name: "cam_right", width: 640, height: 480, fps: 30},
    cam_head:  {type: zmq, server_address: "192.168.0.66", port: 5555, camera_name: "cam_head",  width: 640, height: 480, fps: 30}
  }' \
 --display_mode=rerun
```

> `teleop.ws_url` 默认是 `127.0.0.1`，远程运行时必须显式指定。

网络需放行 TCP **5555**（ZMQ）、**9000**（robot_bridge）。192.168.0.66 上需已启动 `robot_bridge`。