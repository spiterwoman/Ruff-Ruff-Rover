#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_PATH=""
BAGS_ROOT="data/bags"
RATE="1.0"
USE_CLOCK=0
LOOP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-path)
      BAG_PATH="$2"
      shift 2
      ;;
    --bags-root)
      BAGS_ROOT="$2"
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    --clock)
      USE_CLOCK=1
      shift
      ;;
    --loop)
      LOOP=1
      shift
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

resolve_bag_directory() {
  if [[ -n "${BAG_PATH}" ]]; then
    [[ -d "${BAG_PATH}" ]] || {
      echo "Requested bag path does not exist: ${BAG_PATH}" >&2
      exit 1
    }
    echo "${BAG_PATH}"
    return
  fi

  local resolved_root="${REPO_ROOT}/${BAGS_ROOT}"
  [[ -d "${resolved_root}" ]] || {
    echo "Bag root does not exist: ${resolved_root}" >&2
    exit 1
  }

  local latest_bag
  latest_bag="$(find "${resolved_root}" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
  [[ -n "${latest_bag}" ]] || {
    echo "No bag directories were found under: ${resolved_root}" >&2
    exit 1
  }
  echo "${latest_bag}"
}

RESOLVED_BAG_PATH="$(resolve_bag_directory)"

PLAY_ARGS=(bag play "${RESOLVED_BAG_PATH}" --rate "${RATE}")

if [[ "${USE_CLOCK}" -eq 1 ]]; then
  PLAY_ARGS+=(--clock)
fi

if [[ "${LOOP}" -eq 1 ]]; then
  PLAY_ARGS+=(--loop)
fi

echo "Replaying bag: ${RESOLVED_BAG_PATH}"
echo "Running: ros2 ${PLAY_ARGS[*]}"
ros2 "${PLAY_ARGS[@]}"
