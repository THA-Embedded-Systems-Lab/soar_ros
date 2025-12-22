#!/bin/bash

source ~/.bashrc
RUN_ID=$(date -Iseconds)
LOG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/out/${RUN_ID}"
FREQS=(250 500 1000)

NUM_CHANNELS="${1:-3}"

mkdir -p "$LOG_DIR/logs"
mkdir -p "$LOG_DIR/data"
mkdir -p "$LOG_DIR/evaluation"

for F in "${FREQS[@]}"; do
  LOG_FILE_NAME="benchmark_mimo_f_${F}_id_${RUN_ID}.log"
  ros2 launch soar_ros benchmark_mimo.launch.py f:=${F} run_id:=${RUN_ID} num_inputs:=${NUM_CHANNELS} num_outputs:=${NUM_CHANNELS}
  cp ~/.ros/log/latest/launch.log "$LOG_DIR/logs/$LOG_FILE_NAME"
done

python3 evaluation_mimo.py -d "$LOG_DIR" 
