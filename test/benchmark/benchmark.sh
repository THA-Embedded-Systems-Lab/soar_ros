#!/bin/bash

source ~/.bashrc
LOG_DIR="$HOME/tmp/soar_ros_benchmark_logs"
RUN_ID=$(date +%s | sha256sum | cut -c1-16)
FREQS=(250 500 1000)

mkdir -p "$LOG_DIR"

for F in "${FREQS[@]}"; do
  LOG_FILE_NAME="benchmark_f_${F}_id_${RUN_ID}.log"
  ros2 launch soar_ros benchmark.launch.py f:=${F} run_id:=${RUN_ID}
  cp ~/.ros/log/latest/launch.log "$LOG_DIR/$LOG_FILE_NAME"
done

python3 evaluation.py --log_dir "$LOG_DIR" --run_id "$RUN_ID"