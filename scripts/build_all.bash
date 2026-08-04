#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Conda 的 Python 不包含 ROS 2 的系统模块。把系统解释器放在最前面，
# 并显式传给 CMake，保证在显示 (base) 的终端中也能稳定构建。
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME

set +u
source /opt/ros/humble/setup.bash
set -u

build_workspace() {
    local name="$1"
    local workspace="${SYSTEM_ROOT}/${name}"

    printf '\n===== 构建 %s =====\n' "${name}"
    cd "${workspace}"
    colcon build \
        --symlink-install \
        --cmake-force-configure \
        --cmake-args \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_EXECUTABLE=/usr/bin/python3

    set +u
    source "${workspace}/install/setup.bash"
    set -u
}

build_workspace camera_ws
build_workspace chassis_ws
build_workspace aoa_ws
build_workspace rtk_ws
build_workspace perception_ws
build_workspace navigation_ws

printf '\n全部工作空间构建完成。\n'
printf '新终端请执行: source %s/scripts/source_all.bash\n' "${SYSTEM_ROOT}"
