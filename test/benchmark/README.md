# Benchmark Documentation

This directory contains two benchmark configurations for testing the soar_ros system:

## 1. SISO Benchmark (Single Input Single Output)

The original benchmark configuration tests a single input topic → Soar system → single output topic pipeline.

### Components:
- **Sender**: Publishes messages to the `input` topic
- **Receiver**: Subscribes to the `output` topic
- **System**: Soar agent that processes `input` and produces `output`

### Files:
- `Sender.cpp` - SISO sender node
- `Receiver.cpp` - SISO receiver node
- `System.cpp` - SISO Soar system node
- `benchmark.sh` - Script to run SISO benchmark tests
- `evaluation.py` - Analysis script for SISO results
- `../launch/benchmark.launch.py` - SISO launch file
- `../Soar/benchmark.soar` - SISO Soar rules

### Usage:
```bash
# Run the benchmark
cd /home/ws/src/soar_ros/test/benchmark
./benchmark.sh

# Or manually run at a specific frequency:
ros2 launch soar_ros benchmark.launch.py f:=100
```

## 2. MIMO Benchmark (Multiple Input Multiple Output)

Extended benchmark configuration that tests 3 input topics → Soar system → 3 output topics pipeline.

### Components:
- **SenderMIMO**: Publishes messages to `input0`, `input1`, `input2` topics
- **ReceiverMIMO**: Subscribes to `output0`, `output1`, `output2` topics
- **SystemMIMO**: Soar agent that processes multiple inputs and produces multiple outputs

### Files:
- `SenderMIMO.cpp` - MIMO sender node (3 publishers)
- `ReceiverMIMO.cpp` - MIMO receiver node (3 subscribers)
- `SystemMIMO.cpp` - MIMO Soar system node
- `benchmark_mimo.sh` - Script to run MIMO benchmark tests
- `evaluation_mimo.py` - Analysis script for MIMO results
- `../launch/benchmark_mimo.launch.py` - MIMO launch file
- `../Soar/benchmark_mimo.soar` - MIMO Soar rules

### Usage:
```bash
# Run the benchmark
cd /home/ws/src/soar_ros/test/benchmark
./benchmark_mimo.sh

# Or manually run at a specific frequency:
ros2 launch soar_ros benchmark_mimo.launch.py f:=100
```

## Topic Structure

### SISO:
- Input: `input` (std_msgs/msg/Header)
- Output: `output` (std_msgs/msg/Header)

### MIMO:
- Inputs: `input0`, `input1`, `input2` (std_msgs/msg/Header)
- Outputs: `output0`, `output1`, `output2` (std_msgs/msg/Header)

## Message Format

Both benchmarks use the same message type (`std_msgs/msg/Header`):
- `frame_id`: For SISO: string counter (e.g., "0", "1", "2")
- `frame_id`: For MIMO: "counter_index" format (e.g., "0_0", "0_1", "0_2")
- `stamp`: ROS2 timestamp from sender

## Benchmark Parameters

Both scripts test at multiple frequencies:
- 100 Hz
- 250 Hz
- 500 Hz
- 1000 Hz
- 2000 Hz

Each benchmark sends 2000 messages at each frequency.

## Results

Results are saved to:
- SISO: `~/tmp/soar_ros_benchmark_logs/`
- MIMO: `~/tmp/soar_ros_benchmark_mimo_logs/`

The evaluation scripts generate:
- Duration vs frame ID plots
- Frame ID derivative plots (to detect dropped/reordered messages)
- Duration vs receive time plots

## Building

After adding the MIMO files, rebuild the workspace:
```bash
cd /home/ws
colcon build --packages-select soar_ros
source install/setup.bash
```
