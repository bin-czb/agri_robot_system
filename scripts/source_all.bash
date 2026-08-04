#!/usr/bin/env bash

# 本文件需要使用 source 执行，以便环境保留在当前终端。
_AGRI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AGRI_ROBOT_SYSTEM_ROOT="$(cd "${_AGRI_SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash

for _agri_ws in \
    camera_ws \
    chassis_ws \
    aoa_ws \
    rtk_ws \
    perception_ws \
    navigation_ws; do
    _agri_setup="${AGRI_ROBOT_SYSTEM_ROOT}/${_agri_ws}/install/setup.bash"
    if [[ -f "${_agri_setup}" ]]; then
        source "${_agri_setup}"
    else
        printf '提示: %s 尚未构建。\n' "${_agri_ws}" >&2
    fi
done

unset _agri_ws _agri_setup _AGRI_SCRIPT_DIR
