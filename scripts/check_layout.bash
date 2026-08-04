#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

for ws in camera_ws aoa_ws rtk_ws chassis_ws perception_ws navigation_ws; do
    printf '\n===== %s =====\n' "${ws}"
    (cd "${SYSTEM_ROOT}/${ws}" && colcon list)
done

printf '\n检查硬编码旧路径：\n'
if rg -n \
    --glob '*.py' --glob '*.sh' --glob '*.bash' --glob '*.yaml' \
    '/home/czb/pythonProject01|/home/czb/ROS_RTK' \
    "${SYSTEM_ROOT}" \
    --glob '!camera_ws/src/OrbbecSDK_ROS2/**' \
    --glob '!**/scripts/check_layout.bash' \
    --glob '!data/**'; then
    printf '发现旧绝对路径，请审核上面的结果。\n' >&2
    exit 1
fi

printf '目录与旧路径检查通过。\n'
