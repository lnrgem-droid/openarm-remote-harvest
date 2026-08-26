# GitHub 备份与协作

## 第一次上传

建议在 GitHub 创建一个**私有空仓库**，仓库名可用 `openarm-remote-harvest`。创建时不要自动添加 README、`.gitignore` 或 LICENSE，因为本地仓库已经包含这些文件。

创建完成后，在本机执行：

```bash
cd /home/openarm/dev/openarm-remote-harvest
git remote add origin git@github.com:<你的用户名或组织>/openarm-remote-harvest.git
git push -u origin main
git push origin --tags
```

若使用 HTTPS：

```bash
git remote add origin https://github.com/<你的用户名或组织>/openarm-remote-harvest.git
git push -u origin main
git push origin --tags
```

GitHub 不再接受账户密码进行 Git HTTPS 推送，应使用浏览器登录、Personal Access Token 或 SSH Key。

## 建议的分支分工

| 分支 | 负责人/用途 |
| --- | --- |
| `main` | 可回退、经过验证的集成基线 |
| `feature/protocol-contract` | 网络消息、时间戳、状态机和共享内存接口合同 |
| `feature/leader-host` | 主端双臂读取、遥操作界面与 RGB 预览 |
| `feature/jetson-follower` | Jetson 从臂控制、本地看门狗与失效安全 |
| `feature/rgbd-recording` | 三路 Orbbec RGB-D 采集、落盘与时间对齐 |
| `feature/lerobot-converter` | OpenArm 原始数据转换为 LeRobot Dataset |

每个功能通过 Pull Request 合并，不直接在 `main` 上试验。接口合同应优先合并，避免主机端和 Jetson 端各自定义不兼容的消息格式。

## 每次开发前

```bash
git switch main
git pull --ff-only
git switch -c feature/<功能名>
```

完成一个可验证的小步骤后：

```bash
git status
git diff
git add <明确的文件路径>
git commit -m "feat: 简要描述本次改动"
git push -u origin feature/<功能名>
```

## 不应上传的内容

- 相机原始数据、LeRobot Dataset、训练输出和模型权重
- ROS 2 的 `build/install/log`
- Conda/venv 环境
- GitHub Token、SSH 私钥、Wi-Fi 密码、设备账号或 `.env`
- 包含现场隐私且未经确认的录制画面

如果将来确实需要版本化大文件，应先评估 Git LFS 或独立对象存储，不要直接把数据集提交到普通 Git 历史中。

