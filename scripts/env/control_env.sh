#!/usr/bin/env bash

# cobot3_project — Control/FMS development environment
#
# Usage:
#   source scripts/env/control_env.sh
#
# Provides:
#   - ROS 2 Jazzy system environment
#   - project Python virtual environment (.venv)
#   - optional built ROS 2 workspace overlay
#   - common ROS middleware settings

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ---------------------------------------------------------------------------
# ROS 2 Jazzy
# ---------------------------------------------------------------------------

ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS 2 Jazzy setup file not found: $ROS_SETUP"
    return 1 2>/dev/null || exit 1
fi

source "$ROS_SETUP"

# ---------------------------------------------------------------------------
# Project Python virtual environment
# ---------------------------------------------------------------------------

VENV_PATH="$PROJECT_ROOT/.venv"

if [[ ! -f "$VENV_PATH/bin/activate" ]]; then
    echo "[ERROR] Project virtual environment not found:"
    echo "        $VENV_PATH"
    echo
    echo "Create it with:"
    echo "  cd $PROJECT_ROOT"
    echo "  python3 -m venv .venv"
    echo "  source .venv/bin/activate"
    echo "  python -m pip install -r requirements/control.txt"
    return 1 2>/dev/null || exit 1
fi

source "$VENV_PATH/bin/activate"

# ---------------------------------------------------------------------------
# ROS middleware
# ---------------------------------------------------------------------------

export ROS_DOMAIN_ID="${COBOT3_ROS_DOMAIN_ID:-129}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || \
   (( ROS_DOMAIN_ID < 129 || ROS_DOMAIN_ID > 135 )); then
    echo "[ERROR] ROS_DOMAIN_ID must be between 129 and 135."
    echo "        Current value: $ROS_DOMAIN_ID"
    return 1 2>/dev/null || exit 1
fi

# ---------------------------------------------------------------------------
# Local ROS 2 workspace overlay
# ---------------------------------------------------------------------------

WORKSPACE_SETUP="$PROJECT_ROOT/ros2_ws/install/setup.bash"

if [[ -f "$WORKSPACE_SETUP" ]]; then
    source "$WORKSPACE_SETUP"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

echo "========================================"
echo " cobot3_project Control/FMS Environment"
echo "========================================"
echo "PROJECT_ROOT       = $PROJECT_ROOT"
echo "ROS_DISTRO         = $ROS_DISTRO"
echo "ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "PYTHON             = $(command -v python)"
echo "PYTHON_VERSION     = $(python --version 2>&1)"
echo "========================================"
