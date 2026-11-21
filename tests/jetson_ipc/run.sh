#!/usr/bin/env bash
# Inside an isolated Isaac ROS container: run.sh BINARY CAMERA_PREFIX [DETECTOR_PREFIX]
set -eo pipefail
source /opt/ros/lyrical/setup.bash
exe=$(realpath "$1")
camera_prefix=$(realpath "$2")
detector_prefix=$(realpath "${3:-$2}")
for prefix in "$camera_prefix" "$detector_prefix"; do
    test -f "$prefix/lib/libcuda_buffer_backend.so"
done
run_peer() {
    local prefix=$1 mode=$2 limit=$3
    AMENT_PREFIX_PATH="$prefix:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$prefix/lib:$LD_LIBRARY_PATH" \
        timeout "$limit" "$exe" "$mode"
}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
for domain in 0 42; do
    export ROS_DOMAIN_ID=$domain
    logs=$(mktemp -d)
    run_peer "$detector_prefix" sub 18 >"$logs/sub" 2>&1 & sub=$!
    run_peer "$camera_prefix" pub 15 >"$logs/pub" 2>&1 & pub=$!
    rc=0
    wait "$sub" || rc=1
    wait "$pub" || rc=1
    if ((rc != 0)); then
        tail -40 "$logs/sub" "$logs/pub" >&2
        exit 1
    fi
    printf 'ROS domain %d: ' "$domain"
    grep 'Verified CUDA IPC frame' "$logs/sub"
done
