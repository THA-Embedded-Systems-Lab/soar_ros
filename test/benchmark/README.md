# Benchmark Documentation

The benchmark configuration tests a single input topic → Soar system →
single output topic pipeline.

## Components

- **Sender**: Publishes messages to the `input` topic
- **Receiver**: Subscribes to the `output` topic
- **System**: Soar agent that processes `input` and produces `output`

## Usage

```bash
# Run the benchmark
cd /home/ws/src/soar_ros/test/benchmark
./benchmark.sh <channels> <n_messages> <freq1, ... ,freq_n> <auto-delete-bool>
```

## Message Format

The benchmark send `std_msgs/msg/String` messages with a `<channel>_<counter>`
encoding. This allows for multi input and multi output testing in the future.

## How to run

The exact commands to replicate the benchmark configuration as specified in the
publication refer to [evaluation.ipynb](./evaluation.ipynb).

**Important information:** After all messages of a frequency are completed and
there are only cycle messages in the terminal, a manual interrupt of the program
is required in order to start the next frequency test via `CTRL + C`.

## Results

Install all the requried python packages via the uv package manager by running
`uv sync`. Afterwards, runt the Jupyter notebook,
[evaluation.ipynb](./evaluation.ipynb).

## Building

Open the project in VS Code and reopen it in the provided development container.
Once the container starts, the project should build automatically. Rebuild it
with the following command:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The compilation of the Soar library results in some warning messages, but the
build finishes.

## Profiling

See profiling guide of Nav2 stack for detailed information:
<https://docs.nav2.org/tutorials/docs/get_profile.html>

Use the devcontainer, or install required components via:

```shell
sudo apt-get install valgrind kcachegrind
```

Run the profiling via the following two terminals:

Terminal 1:

```shell
valgrind --tool=callgrind ./install/soar_ros/lib/soar_ros/System
```

Wait until you see the decision cycle execution debug message and launch this in
terminal 2:

```shell
ros2 launch soar_ros benchmark-send-receive.profiling.launch.py
```

Analyze the output via

```shell
kcachegrind callgrind.out.<number>
```
