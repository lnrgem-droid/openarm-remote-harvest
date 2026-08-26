# openarm_move_pose

薄封装：位姿控制 + 末端位姿反馈。依赖底层 `ros2_control` + `move_group`（或 `demo.launch.py`）。

## 启动

```bash
# 先起控制栈 + move_group，再：
ros2 launch openarm_move_pose move_pose.launch.py
# 可选：rate:=10.0
```

## 接口


| 名称           | 类型                                  | 说明                            |
| ------------ | ----------------------------------- | ----------------------------- |
| `/move_pose` | Action `openarm_move_pose/MovePose` | 位姿规划/执行 → 内部转发 `/move_action` |
| `/ee_poses`  | `tf2_msgs/TFMessage`                | 左右臂末端位姿，默认 5 Hz               |


### `/move_pose`

- Goal：`arm`（`left`/`right`）、`pose`、`frame_id`（空=`world`）、`vel_scale`/`acc_scale`（≤0 默认 0.2）、`plan_only`
- 末端：`left`→`openarm_left_hand`，`right`→`openarm_right_hand`

```bash
ros2 action send_goal /move_pose openarm_move_pose/action/MovePose "{
  arm: 'left',
  pose: {
    position: {x: 0.15, y: 0.15, z: 0.16},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  },
  vel_scale: 0.2,
  acc_scale: 0.2,
  plan_only: false
}"
```



### `/ee_poses`

每条 `transforms[]`：

- `header.frame_id` → `child_frame_id`：`world` → `openarm_left_hand` / `openarm_right_hand`
- `transform`：xyz + 四元数

```bash
ros2 topic echo /ee_poses
```

参数：`rate`（默认 5）、`parent_frame`（默认 `world`）、`topic`（默认 `ee_poses`）。