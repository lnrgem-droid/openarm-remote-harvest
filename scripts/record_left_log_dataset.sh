#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/nvidia/openarm-left-only-20261113/scripts/left_only_env.sh
export PYTHONNOUSERSITE=1
exec taskset --cpu-list 3-7 nice -n 10 "$ROOT/.venv/bin/python" "$ROOT/scripts/record_left_log.py" --root "${DATASET_ROOT:?}" --task "${TASK:?}"
