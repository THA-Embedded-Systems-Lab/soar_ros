# Benchmark Documentation

This directory contains two benchmark configurations for testing the soar_ros system:

## Benchmark

The original benchmark configuration tests a single input topic → Soar system →
single output topic pipeline and multi input to multi output.

### Components

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

## Topic Structure

## Message Format

Both benchmarks use the same message type (`std_msgs/msg/Header`):

see sender system and receiver.cpp
times are based on log messages.

## Benchmark Parameters

see benchmark.sh and benchmark.launch.py

## How to run

See evaluation.ipynb

## Results

See Jupyter notebook

## Building

Open the project in VS Code and reopen it in the provided development container.
Once the container starts, the project should build automatically. Rebuild it
with the following command:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The copmilation of the soar library results in some warning messags, but the
build finishes.

## Profiling

See profiling guide of Nav2 stack for detailed information: https://docs.nav2.org/tutorials/docs/get_profile.html

Use the devcontinaer, or install required components via:

```shell
sudo apt-get install valgrind kcachegrind
```

Run the profiling via the following two terminals:

Terminal 1:

```shell
valgrind --tool=callgrind ./install/soar_ros/lib/soar_ros/System
```

Wait until you see the decision cycle execution debug message and launch this in terminal 2:

```shell
ros2 launch soar_ros benchmark-send-receive.profiling.launch.py
```

Analyze the output via

```shell
kcachegrind callgrind.out.<number>
```

## Debug

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2: ROS 2 Launch",
            "request": "launch",
            "target": "/home/ws/install/soar_ros/share/soar_ros/launch/benchmark.launch.py",
            "launch": [
                "rviz",
                "gz",
                "Sender",
                "Receiver"
            ],
            "type": "ros2",
            "arguments": [
                "debug:=True"
            ],
        }
    ]
}
```