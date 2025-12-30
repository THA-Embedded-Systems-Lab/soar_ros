#!/bin/bash

source ~/.bashrc

NUM_CHANNELS="${1:-3}"

RUN_ID="$(date -Iseconds | cut -d'+' -f1)_CHANNELS_${NUM_CHANNELS}"
LOG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/out/${RUN_ID}"
FREQS=(125 250 500)

mkdir -p "$LOG_DIR/logs"
mkdir -p "$LOG_DIR/data"
mkdir -p "$LOG_DIR/evaluation"

for F in "${FREQS[@]}"; do
  LOG_FILE_NAME="benchmark_f_${F}.log"
  ros2 launch soar_ros benchmark.launch.py f:=${F} run_id:=${RUN_ID} num_inputs:=${NUM_CHANNELS} num_outputs:=${NUM_CHANNELS} 
  cp ~/.ros/log/latest/launch.log "$LOG_DIR/logs/$LOG_FILE_NAME"
done

echo python3 parse-logs.py -d "$LOG_DIR"
python3 parse-logs.py -d "$LOG_DIR"

echo "Benchmarking complete. Analyze data in Jupyter Notebook located in test/benchmark/evaluation.ipynb"
