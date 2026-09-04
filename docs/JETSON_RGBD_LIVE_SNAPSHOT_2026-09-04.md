# Jetson RGB-D 部署快照（2026-09-04）

此归档分支基于 `feat/jetson-rgbd-preview`，并覆盖为 Jetson 实际部署目录中与功能分支内容不同的三个源码/文档文件：

- `README.md`
- `scripts/rgb_preview_live.py`
- `scripts/rgbd_collection_console.py`

其余差异为 Python 缓存、egg-info 或运行时生成物，未纳入版本控制。Jetson 的 RGB-D 部署目录没有 `.git`，因此本分支用于准确保留当日可见的部署源码，供审查和后续合并比对。
