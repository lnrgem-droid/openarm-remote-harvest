#!/usr/bin/env bash
# Run after the RGB-D service and read-only bridge are healthy.
# This script records the follower state + applied external action + six RGB-D streams.
set -euo pipefail

ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
PYTHON_BIN="${LEROBOT_PYTHON:-/home/nvidia/miniconda3/envs/lerobot/bin/python}"
DATASET_ROOT="${DATASET_ROOT:-/home/nvidia/datasets/openarm_rgbd_$(date +%Y%m%d_%H%M%S)}"
DATASET_ID="${DATASET_ID:-openarm/mushroom-rgbd}"
TASK="${TASK:-bimanual mushroom harvesting teleoperation}"
IMAGE_WRITER_PROCESSES="${IMAGE_WRITER_PROCESSES:-1}"
IMAGE_WRITER_THREADS="${IMAGE_WRITER_THREADS:-2}"
EPISODE_TIME_S="${EPISODE_TIME_S:-60}"
RESET_TIME_S="${RESET_TIME_S:-60}"
NUM_EPISODES="${NUM_EPISODES:-50}"

DATASET_PARENT=$(dirname "$DATASET_ROOT")
mkdir -p "$DATASET_PARENT"
if [[ -e "$DATASET_ROOT" ]]; then
  echo "Refusing recording: dataset path already exists: $DATASET_ROOT" >&2
  echo "Choose a new DATASET_ROOT; LeRobot requires a new empty path." >&2
  exit 1
fi
available_gb=$(df -PB1G "$DATASET_PARENT" | awk 'NR==2 {gsub("G", "", $4); print $4}')
if (( available_gb < 20 )); then
  echo "Refusing recording: only ${available_gb} GB free; at least 20 GB is required." >&2
  exit 1
fi

exec "$PYTHON_BIN" -m lerobot.scripts.lerobot_record \
  --robot.type=openarm_bridge \
  --robot.control_authority=external \
  --robot.ws_url=ws://127.0.0.1:9000 \
  --robot.rgbd_endpoint=ipc:///tmp/openarm_rgbd_raw.ipc \
  --teleop.type=openarm_bridge_teleop \
  --teleop.ws_url=ws://127.0.0.1:9000 \
  --dataset.repo_id="$DATASET_ID" \
  --dataset.root="$DATASET_ROOT" \
  --dataset.single_task="$TASK" \
  --dataset.fps=30 --dataset.episode_time_s="$EPISODE_TIME_S" \
  --dataset.reset_time_s="$RESET_TIME_S" --dataset.num_episodes="$NUM_EPISODES" \
  --dataset.video=true --dataset.push_to_hub=false \
  --dataset.streaming_encoding=false --dataset.video_encoding_batch_size=2 \
  --dataset.num_image_writer_processes="$IMAGE_WRITER_PROCESSES" \
  --dataset.num_image_writer_threads_per_camera="$IMAGE_WRITER_THREADS" \
  --dataset.encoder_threads=2 \
  --dataset.rgb_encoder.vcodec=h264 --dataset.rgb_encoder.pix_fmt=yuv420p \
  --dataset.rgb_encoder.crf=23 --dataset.rgb_encoder.preset=veryfast \
  --dataset.depth_encoder.vcodec=hevc --dataset.depth_encoder.pix_fmt=gray12le \
  --dataset.depth_encoder.crf=0 --dataset.depth_encoder.preset=ultrafast
