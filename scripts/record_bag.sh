#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

OUTPUT_ROOT="data/bags"
BAG_NAME=""
STORAGE_ID="sqlite3"
ALL_TOPICS=0
TOPICS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --bag-name)
      BAG_NAME="$2"
      shift 2
      ;;
    --storage-id)
      STORAGE_ID="$2"
      shift 2
      ;;
    --all-topics)
      ALL_TOPICS=1
      shift
      ;;
    --topic)
      TOPICS+=("$2")
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 was not found in PATH. Source your ROS 2 environment first." >&2
  exit 1
}

mkdir -p "${REPO_ROOT}/${OUTPUT_ROOT}"

if [[ -z "${BAG_NAME}" ]]; then
  BAG_NAME="$(date +%Y%m%d_%H%M%S)"
fi

BAG_PATH="${REPO_ROOT}/${OUTPUT_ROOT}/${BAG_NAME}"

DEFAULT_TOPICS=(
  /tf
  /tf_static
  /odom
  /wheel_state
  /cmd_vel
  /pico/heartbeat
  /range/front_left
  /range/front_right
  /camera/ready
  /whistle/event
  /whistle/ready
  /target_track
  /vision/ready
  /self_test/ready
  /self_test/status
  /behavior/state
)

RECORD_ARGS=(bag record -s "${STORAGE_ID}" -o "${BAG_PATH}")

if [[ "${ALL_TOPICS}" -eq 1 ]]; then
  RECORD_ARGS+=(-a)
else
  if [[ "${#TOPICS[@]}" -eq 0 ]]; then
    TOPICS=("${DEFAULT_TOPICS[@]}")
  fi
  RECORD_ARGS+=("${TOPICS[@]}")
fi

echo "Recording bag to: ${BAG_PATH}"
echo "Running: ros2 ${RECORD_ARGS[*]}"
ros2 "${RECORD_ARGS[@]}"
