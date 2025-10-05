# Benchmark Test Documentation

## 1. Scope

This benchmark evaluates the message passing performance and reliability of the
soar_ros system using ROS 2. It measures the latency and consistency of header
message delivery between a sender and receiver node at various frequencies, and
checks for dropped or out-of-order messages.

## 2. Setup

- ROS 2 nodes: Sender, Receiver, System
- Sender publishes `std_msgs::msg::Header` messages to the `input` topic at configurable frequencies.
- Receiver subscribes to the `output` topic and logs received messages with timestamps.
- System node processes messages.
- The number of messages sent per run is configurable (default: 2000).

```mermaid
flowchart LR
    Sender --> System --> Receiver
```

## 3. How to run

1. Build the workspace with benchmark executables enabled.
2. Use the provided [`benchmark.sh`](./../test/benchmark/benchmark.sh) script to
launch the benchmark for multiple frequencies:

   ```bash
   ./benchmark.sh
   ```
   - This will run tests for each frequency and save logs in `~/.ros/log`.
   - Make sure to `CTRL + C` **ONCE** after every frequency, since nodes do not shut down automatically. 
    The following log messages indicate one frequency test completed:

        ```log
        [Sender-1] [INFO] [1759677870.464450522] [sender]: Sent 2000 messages. Shutting down.
        [INFO] [Sender-1]: process has finished cleanly [pid 85312]
        ```
    - The next test will start immediatly without further actions required.

3. After runs complete, results are analyzed automatically with the evaluation script:

   ```bash
   python evaluation.py --log_dir ~/tmp/soar_ros_benchmark_log/ --run_id <your_run_id>
   ```
   - This script generates plots and prints dropped/out-of-order message info.

4. Results are saved to '~/tmp/soar_ros_benchmark_logs/'

## 4. Results

- The evaluation script produces:
  - Latency plots: duration between sender and receiver vs. frame_id and vs. time.
  - Message order plots: derivative of frame_id series to check for consistency.
  - Dropped message report: lists any missing frame_ids.
- Use these results to assess system performance at different message rates and
identify bottlenecks or reliability issues.

![](./Images/duration_vs_frameid.png)
![](./Images/duration_vs_time.png)

## 5. Relevant Files

- [Sender.cpp](../test/benchmark/Sender.cpp): Publishes header messages at a configurable frequency to the input topic.
- [Receiver.cpp](../test/benchmark/Receiver.cpp): Subscribes to the output topic and logs received header messages with timestamps.
- [System.cpp](../test/benchmark/System.cpp): The soar_ros system under test, processes input and output messages.
- [benchmark.launch.py](../launch/benchmark.launch.py): Launch file to start sender, receiver, and system nodes with configurable parameters and logging.
- [benchmark.sh](../test/benchmark/benchmark.sh): Shell script to automate running the benchmark for multiple frequencies and run IDs.
- [evaluation.py](../test/benchmark/evaluation.py): Python script to analyze log files, check for dropped/out-of-order messages, and generate performance plots.
- [benchmark.soar](../Soar/benchmark.soar): Soar rules for message handling and status marking in the system under test.