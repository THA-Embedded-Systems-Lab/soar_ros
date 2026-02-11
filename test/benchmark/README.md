# Benchmark Documentation

This directory contains two benchmark configurations for testing the soar_ros system:


## Benchmark

The original benchmark configuration tests a single input topic → Soar system →
single output topic pipeline and multi input to multi output.

### Components

-   **Sender**: Publishes messages to the `input` topic
-   **Receiver**: Subscribes to the `output` topic
-   **System**: Soar agent that processes `input` and produces `output`

### Files:

-   `Sender.cpp` - SISO sender node
-   `Receiver.cpp` - SISO receiver node
-   `System.cpp` - SISO Soar system node
-   `benchmark.sh` - Script to run SISO benchmark tests
-   `evaluation.py` - Analysis script for SISO results
-   `../launch/benchmark.launch.py` - SISO launch file
-   `../Soar/benchmark.soar` - SISO Soar rules

### Usage:

```bash
# Run the benchmark
cd /home/ws/src/soar_ros/test/benchmark
./benchmark.sh

# Or manually run at a specific frequency:
ros2 launch soar_ros benchmark.launch.py f:=100
```


## Topic Structure


## Message Format

Both benchmarks use the same message type (`std_msgs/msg/Header`):

see sender system and receiver.cpp
times are based on log messages.

## Benchmark Parameters

Both scripts test at multiple frequencies:

see benchmark.sh

## Results

See Jupyter notebook

## Building


```bash
cd /home/ws
colcon build --packages-select soar_ros --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Profiling

See profiling guide of Nav2 stack for detailed information: https://docs.nav2.org/tutorials/docs/get_profile.html

```shell
sudo apt-get install valgrind kcachegrind
```

terminal 1:

```shell
valgrind --tool=callgrind ./install/soar_ros/lib/soar_ros/System 
```


Wait until you see the decision cycle execution debug message and launch this in terminal 2:

```shell
ros2 launch soar_ros benchmark-send-receive.profiling.launch.py
```