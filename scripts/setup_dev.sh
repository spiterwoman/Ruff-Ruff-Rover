#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_SETUP_PATH="${ROS_SETUP_PATH:-}"
BUILD_TYPE="RelWithDebInfo"
SKIP_BUILD=0
PACKAGES_SELECT=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-setup-path)
      ROS_SETUP_PATH="$2"
      shift 2
      ;;
    --build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --packages-select)
      shift
      while [[ $# -gt 0 ]] && [[ ! "$1" =~ ^-- ]]; do
        PACKAGES_SELECT+=("$1")
        shift
      done
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

find_ros_setup() {
  if [[ -n "${ROS_SETUP_PATH}" ]]; then
    [[ -f "${ROS_SETUP_PATH}" ]] || {
      echo "ROS setup path does not exist: ${ROS_SETUP_PATH}" >&2
      exit 1
    }
    echo "${ROS_SETUP_PATH}"
    return
  fi

  local candidates=(
    "/opt/ros/jazzy/setup.bash"
    "/opt/ros/humble/setup.bash"
    "/opt/ros/iron/setup.bash"
    "/opt/ros/rolling/setup.bash"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      echo "${candidate}"
      return
    fi
  done
}

echo "Workspace root: ${REPO_ROOT}"

RESOLVED_ROS_SETUP="$(find_ros_setup || true)"
if [[ -n "${RESOLVED_ROS_SETUP}" ]]; then
  echo "Sourcing ROS 2 environment: ${RESOLVED_ROS_SETUP}"
  # shellcheck disable=SC1090
  source "${RESOLVED_ROS_SETUP}"
else
  echo "Warning: no ROS 2 setup script was found automatically." >&2
  echo "Build will continue only if ROS 2 is already available in this shell." >&2
fi

command -v colcon >/dev/null 2>&1 || {
  echo "colcon was not found in PATH. Install ROS 2 tooling or start from a ROS-enabled shell." >&2
  exit 1
}

cd "${REPO_ROOT}"

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  BUILD_ARGS=(
    build
    --symlink-install
    --cmake-args
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
  )

  if [[ "${#PACKAGES_SELECT[@]}" -gt 0 ]]; then
    BUILD_ARGS+=(--packages-select)
    BUILD_ARGS+=("${PACKAGES_SELECT[@]}")
  fi

  echo "Running: colcon ${BUILD_ARGS[*]}"
  colcon "${BUILD_ARGS[@]}"
fi

if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo
  echo "Build complete."
  echo "To use the workspace in your current shell, run:"
  echo "source \"${REPO_ROOT}/install/setup.bash\""
else
  echo "Warning: workspace overlay not found at ${REPO_ROOT}/install/setup.bash" >&2
fi
