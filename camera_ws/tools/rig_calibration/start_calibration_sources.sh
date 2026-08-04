#!/bin/bash

# Start the Orbbec RGB-D/synchronized-IMU source and, optionally, the raw AOA
# measurement node.  Run rig_calibration.py from a second terminal.

MODE="${1:-all}"
AOA_PORT="${2:-/dev/ttyACM0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEM_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ "$MODE" != "all" ] && [ "$MODE" != "camera" ] && [ "$MODE" != "aoa" ]; then
    echo "Usage: $0 [all|camera|aoa] [/dev/ttyACM0]" >&2
    exit 2
fi

source "$SYSTEM_ROOT/scripts/source_all.bash"

PIDS=()
cleanup() {
    echo "Stopping calibration sources..."
    if [ "${#PIDS[@]}" -gt 0 ]; then
        kill "${PIDS[@]}" 2>/dev/null
        wait "${PIDS[@]}" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

if [ "$MODE" = "all" ] || [ "$MODE" = "camera" ]; then
    echo "Starting Orbbec RGB-D and synchronized 200 Hz IMU..."
    ros2 launch agri_vslam_bringup orbbec_gemini_slam.launch.py &
    PIDS+=("$!")
    sleep 5
fi

if [ "$MODE" = "all" ] || [ "$MODE" = "aoa" ]; then
    if [ ! -e "$AOA_PORT" ]; then
        echo "AOA serial port not found: $AOA_PORT" >&2
        exit 3
    fi
    echo "Starting raw AOA calibration source on $AOA_PORT..."
    ros2 run agri_aoa_driver aoa_localization_node --ros-args \
        -p serial_port:="$AOA_PORT" \
        -p base_heading_mode:=fixed_east \
        -p base_fixed_yaw_enu_deg:=0.0 \
        -p aoa_range_alpha:=1.0 \
        -p aoa_angle_alpha:=1.0 \
        -p aoa_opt_enable:=false &
    PIDS+=("$!")
fi

echo "Calibration sources are running. Press Ctrl+C here after all stages finish."
wait
