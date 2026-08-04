#!/usr/bin/env bash

set -Eeuo pipefail

DEFAULT_OUTPUT_ROOT="${HOME}/agri_camera_bags"
OUTPUT_ROOT="${DEFAULT_OUTPUT_ROOT}"
BAG_NAME="rgbd_imu_$(date +%Y%m%d_%H%M%S)"
DURATION=0
WAIT_TIMEOUT=30
USE_COMPRESSION=true

usage() {
  cat <<'EOF'
采集 Gemini 335L 的 RGB、注册深度、相机内参、IMU 和 TF。

用法:
  record_rgbd_imu_bag.sh [选项]

选项:
  --output-dir DIR    数据根目录，默认: ~/agri_camera_bags
  --name NAME         rosbag 目录名，默认: rgbd_imu_年月日_时分秒
  --duration SEC      自动停止秒数；0 表示按 Ctrl+C 停止，默认: 0
  --wait-timeout SEC  等待相机有效数据的秒数，默认: 30
  --no-compression    不使用 zstd 文件压缩
  -h, --help          显示帮助

示例:
  record_rgbd_imu_bag.sh --name orchard_01
  record_rgbd_imu_bag.sh --duration 120 --name indoor_test
EOF
}

while (($# > 0)); do
  case "$1" in
    --output-dir)
      OUTPUT_ROOT="${2:?--output-dir 需要目录参数}"
      shift 2
      ;;
    --name)
      BAG_NAME="${2:?--name 需要名称参数}"
      shift 2
      ;;
    --duration)
      DURATION="${2:?--duration 需要秒数参数}"
      shift 2
      ;;
    --wait-timeout)
      WAIT_TIMEOUT="${2:?--wait-timeout 需要秒数参数}"
      shift 2
      ;;
    --no-compression)
      USE_COMPRESSION=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误: 未知参数 $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION}" =~ ^[0-9]+$ ]] || ! [[ "${WAIT_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误: --duration 必须是非负整数，--wait-timeout 必须是正整数。" >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误: 找不到 ros2。请先 source ROS 2、Orbbec SDK 和工作空间环境。" >&2
  exit 1
fi

if [[ "${BAG_NAME}" == */* || "${BAG_NAME}" == "." || "${BAG_NAME}" == ".." ]]; then
  echo "错误: --name 只能是目录名，不能包含斜杠。" >&2
  exit 2
fi

REQUIRED_TOPICS=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
  /camera/gyro_accel/sample
)

RECORD_TOPICS=(
  "${REQUIRED_TOPICS[@]}"
  /tf
  /tf_static
)

publisher_count() {
  ros2 topic info "$1" 2>/dev/null |
    awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

echo "正在等待相机话题及发布者，最长 ${WAIT_TIMEOUT} 秒..."
deadline=$((SECONDS + WAIT_TIMEOUT))
while true; do
  missing=()
  for topic in "${REQUIRED_TOPICS[@]}"; do
    count="$(publisher_count "${topic}")"
    if [[ ! "${count}" =~ ^[0-9]+$ ]] || ((count < 1)); then
      missing+=("${topic}")
    fi
  done

  if ((${#missing[@]} == 0)); then
    break
  fi

  if ((SECONDS >= deadline)); then
    echo "错误: 以下必需话题在 ${WAIT_TIMEOUT} 秒内没有发布者:" >&2
    printf '  %s\n' "${missing[@]}" >&2
    echo "请确认已启动 orbbec_gemini_slam.launch.py，且相机没有被其他进程占用。" >&2
    exit 1
  fi
  sleep 1
done

echo "发布者检查通过，继续检查每个传感器话题是否收到有效消息..."
for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! timeout 6s ros2 topic echo \
      --once \
      --qos-reliability best_effort \
      --qos-durability volatile \
      "${topic}" >/dev/null 2>&1; then
    echo "错误: ${topic} 有发布者，但 6 秒内没有收到消息，已取消录制。" >&2
    exit 1
  fi
  echo "  [正常] ${topic}"
done

OPTIONAL_TOPICS=(
  /camera/imu/calibrated_raw
  /camera/imu/data
)
for topic in "${OPTIONAL_TOPICS[@]}"; do
  count="$(publisher_count "${topic}")"
  if [[ "${count}" =~ ^[0-9]+$ ]] && ((count > 0)); then
    RECORD_TOPICS+=("${topic}")
    echo "  [附加] ${topic}"
  fi
done

mkdir -p "${OUTPUT_ROOT}"
BAG_PATH="${OUTPUT_ROOT}/${BAG_NAME}"
if [[ -e "${BAG_PATH}" ]]; then
  echo "错误: 输出路径已经存在: ${BAG_PATH}" >&2
  echo "请更换 --name，避免覆盖已有数据。" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_QOS="${SCRIPT_DIR}/../config/rosbag2_camera_qos.yaml"
INSTALLED_QOS="${SCRIPT_DIR}/../../share/agri_vslam_bringup/config/rosbag2_camera_qos.yaml"
if [[ -f "${SOURCE_QOS}" ]]; then
  QOS_FILE="$(realpath "${SOURCE_QOS}")"
elif [[ -f "${INSTALLED_QOS}" ]]; then
  QOS_FILE="$(realpath "${INSTALLED_QOS}")"
else
  echo "错误: 找不到 rosbag2_camera_qos.yaml，请重新编译 agri_vslam_bringup。" >&2
  exit 1
fi

RECORD_CMD=(
  ros2 bag record
  --storage sqlite3
  --output "${BAG_PATH}"
  --max-cache-size 268435456
  --qos-profile-overrides-path "${QOS_FILE}"
)
if [[ "${USE_COMPRESSION}" == true ]]; then
  RECORD_CMD+=(
    --compression-mode file
    --compression-format zstd
    --compression-queue-size 2
    --compression-threads 2
  )
fi
RECORD_CMD+=("${RECORD_TOPICS[@]}")

printf '%s\n' \
  "========================================" \
  "开始采集 RGB-D + IMU 建图数据" \
  "输出目录: ${BAG_PATH}" \
  "录制时长: $([[ ${DURATION} -gt 0 ]] && echo "${DURATION} 秒" || echo "手动停止")" \
  "停止方式: Ctrl+C，并等待 rosbag 正常写完 metadata.yaml" \
  "========================================"
printf '  %s\n' "${RECORD_TOPICS[@]}"

if ((DURATION > 0)); then
  set +e
  timeout --signal=INT --kill-after=15s "${DURATION}s" "${RECORD_CMD[@]}"
  status=$?
  set -e
  if ((status != 0 && status != 124)); then
    echo "错误: rosbag 录制异常退出，状态码 ${status}。" >&2
    exit "${status}"
  fi
else
  "${RECORD_CMD[@]}"
fi

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "错误: 未生成 metadata.yaml，数据包可能没有正常收尾。" >&2
  exit 1
fi

echo
echo "采集完成: ${BAG_PATH}"
echo "检查命令:"
echo "  ros2 bag info ${BAG_PATH}"
