#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Installed scripts live in install/bin; the config and executable live one
# directory above it.
if [ -x "$SCRIPT_DIR/../aimrt_main" ] && [ -f "$SCRIPT_DIR/../cfg/x1_cfg_sim.yaml" ]; then
    BUILD_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
elif [ -x "$SCRIPT_DIR/aimrt_main" ] && [ -f "$SCRIPT_DIR/cfg/x1_cfg_sim.yaml" ]; then
    BUILD_DIR="$SCRIPT_DIR"
else
    echo "Cannot locate aimrt_main and cfg/x1_cfg_sim.yaml relative to $SCRIPT_DIR" >&2
    exit 1
fi
PROJECT_DIR="$(cd "$BUILD_DIR/.." && pwd)"

export LD_LIBRARY_PATH="${CONDA_PREFIX:+$CONDA_PREFIX/lib:}$PROJECT_DIR/src/module/joy_stick_module/third_party/lib:$BUILD_DIR/install/lib:${LD_LIBRARY_PATH:-}"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

cd "$BUILD_DIR"
exec "$BUILD_DIR/aimrt_main" --cfg_file_path=./cfg/x1_cfg_sim.yaml
