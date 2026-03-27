#!/bin/bash

source ~/.bashrc

# Usage: ./benchmark.sh [NUM_CHANNELS] [NUM_MESSAGES] [FREQS] [AUTO_DELETE_SOAR_IO]
#
# Parameters:
#   NUM_CHANNELS       - Number of input/output channels (default: 1)
#   NUM_MESSAGES       - Number of messages to send (default: 2000)
#   FREQS              - Comma-separated list of frequencies (default: 1000,1200,1500)
#   AUTO_DELETE_SOAR_IO - Whether to auto-delete SOAR IO on completion (default: False)

NUM_CHANNELS="${1:-1}"
NUM_MESSAGES="${2:-2000}"
IFS=',' read -r -a FREQS <<< "${3:-1000,1200,1500}"
AUTO_DELETE_SOAR_IO="${4:-False}"

RUN_ID="$(date -Iseconds | cut -d'+' -f1)_CHANNELS_${NUM_CHANNELS}"
LOG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/out/${RUN_ID}"

mkdir -p "$LOG_DIR/logs"
mkdir -p "$LOG_DIR/data"
mkdir -p "$LOG_DIR/evaluation"

for F in "${FREQS[@]}"; do
  LOG_FILE_NAME="benchmark_f_${F}.log"
  ros2 launch soar_ros benchmark.launch.py f:=${F} num_inputs:=${NUM_CHANNELS} num_outputs:=${NUM_CHANNELS} messages_to_send:=${NUM_MESSAGES} auto_delete_soar_io_on_complete:=${AUTO_DELETE_SOAR_IO}
  cp ~/.ros/log/latest/launch.log "$LOG_DIR/logs/$LOG_FILE_NAME"
done

echo python3 parse-logs.py -d "$LOG_DIR"
python3 parse-logs.py -d "$LOG_DIR"

echo "Benchmarking complete. Analyze data in Jupyter Notebook located in test/benchmark/evaluation.ipynb"
