#!/bin/bash

source ~/.bashrc

# Runs all benchmark test cases unattended.
# Each ros2 launch session is automatically terminated after a computed timeout:
#
#   t_timeout = ceil(NUM_MESSAGES / FREQ * SAFETY_FACTOR + OFFSET_S)
#
# Excess data collected after all messages are sent is removed during post-processing.
#
# Note: The "preferences" scenario requires the benchmark-preferences branch and
#       must be run separately.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SAFETY_FACTOR=2
OFFSET_S=10

SESSION_ID="$(date -Iseconds | cut -d'+' -f1)"

# run_benchmark <NUM_CHANNELS> <NUM_MESSAGES> <FREQS_CSV> <AUTO_DELETE> <PREFERENCES_BASED_SORT> [MIN_TIMEOUT]
run_benchmark() {
  local NUM_CHANNELS="$1"
  local NUM_MESSAGES="$2"
  local AUTO_DELETE="$4"
  local PREFERENCES_BASED_SORT="$5"
  local MIN_TIMEOUT="${6:-0}"

  IFS=',' read -r -a FREQS <<< "$3"

  local FREQS_LABEL="${3//,/-}"
  local RUN_ID="${SESSION_ID}_ch${NUM_CHANNELS}_msg${NUM_MESSAGES}_f${FREQS_LABEL}_del${AUTO_DELETE}_sort${PREFERENCES_BASED_SORT}"
  local LOG_DIR="$SCRIPT_DIR/out/${RUN_ID}"

  mkdir -p "$LOG_DIR/logs"
  mkdir -p "$LOG_DIR/data"
  mkdir -p "$LOG_DIR/evaluation"

  for F in "${FREQS[@]}"; do
    local TIMEOUT
    TIMEOUT=$(awk "BEGIN { v = ${NUM_MESSAGES} / ${F} * ${SAFETY_FACTOR} + ${OFFSET_S}; t = (v == int(v)) ? v : int(v)+1; print (t > ${MIN_TIMEOUT}) ? t : ${MIN_TIMEOUT} }")
    # TIMEOUT=5 # For debugging the script.

    echo ""
    echo "=== channels=${NUM_CHANNELS}, messages=${NUM_MESSAGES}, freq=${F} Hz, auto_delete=${AUTO_DELETE}, preferences_based_sort=${PREFERENCES_BASED_SORT}, timeout=${TIMEOUT}s  ==="
    timeout "$TIMEOUT" ros2 launch soar_ros benchmark.launch.py \
      f:="${F}" \
      num_inputs:="${NUM_CHANNELS}" \
      num_outputs:="${NUM_CHANNELS}" \
      messages_to_send:="${NUM_MESSAGES}" \
      auto_delete_soar_io_on_complete:="${AUTO_DELETE}" \
      preferences_based_sort:="${PREFERENCES_BASED_SORT}" || true

    cp ~/.ros/log/latest/launch.log "$LOG_DIR/logs/benchmark_f_${F}.log"

    # Ensure all system nodes are down.
    pkill -fe -9 soar_ros/System
    pkill -fe -9 soar_ros/Receiver
    pkill -fe -9 soar_ros/Sender
    # echo "Waiting for 5 seconds before next run..." 
    sleep 5
  done

  echo "Parsing logs..."
  python3 "$SCRIPT_DIR/parse-logs.py" -d "$LOG_DIR"
  echo "Results saved to: $LOG_DIR"
}

# --- Test Cases ---

# 1. No Load: Soar kernel baseline with no I/O channels
run_benchmark 0 0 "1000" "True" "False"

# 2. SISO: Single-channel throughput sweep with auto-delete
run_benchmark 1 4000 "2500,3000" "True" "False" 80

# 3. Normal: Low-frequency, WME accumulation (no auto-delete)
run_benchmark 1 8000 "200" "False" "False"

# 4. Auto-Delete: Low-frequency with WME cleanup enabled
run_benchmark 1 8000 "200" "True" "False"

# 5. Preferences-Based Sort: Low-frequency with auto-delete and preferences-based sorting
run_benchmark 1 100 "2500,3000" "True" "True" 12

echo ""
echo "All benchmarks complete. SESSION_ID: ${SESSION_ID}"
echo "executing..." python3 "$SCRIPT_DIR/evaluation.py" "$SESSION_ID"
python3 "$SCRIPT_DIR/evaluation.py" "$SESSION_ID"
