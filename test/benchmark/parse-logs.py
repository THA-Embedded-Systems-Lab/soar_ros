import os
import re
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def parse_log_file(filepath):
    sender_pattern = re.compile(r'Published to input: frame_id=([^,]+)')
    receiver_pattern = re.compile(r'Received on output: frame_id=([^,]+)')
    soar_decision_pattern = re.compile(r'Soar decision cycle executed')
    # Example log: [INFO] [1766742660.710005840] [SoarRunner]: Soar decision cycle executed
    log_prefix_pattern = re.compile(r'\[(?:INFO|DEBUG|WARN|ERROR)\] \[(\d+\.\d+)\]')

    sender_msgs = {}  # {(frame_id, input_index): timestamp}
    receiver_msgs = []
    soar_decision_records = []


    with open(filepath, 'r') as f:
        for line in f:
            sender_match = sender_pattern.search(line)
            if sender_match:
                counter = int(sender_match.group(1).split("_")[0])
                channel = int(sender_match.group(1).split("_")[1])
                # capture log prefix timestamp for this sender line if present
                prefix_match = log_prefix_pattern.search(line)
                log_time_sender = float(prefix_match.group(1))
                sender_msgs[(counter, channel)] = {
                    'sender_time': log_time_sender,
                }

            receiver_match = receiver_pattern.search(line)
            if receiver_match:
                counter = int(receiver_match.group(1).split("_")[0])
                channel = int(receiver_match.group(1).split("_")[1])
                prefix_match = log_prefix_pattern.search(line)
                log_time_receiver = float(prefix_match.group(1))
                receiver_msgs.append({
                    'counter': counter,
                    'channel': channel,
                    'receive_time': log_time_receiver,
                })

            # Collect Soar decision cycle messages with timestamp from log prefix
            if soar_decision_pattern.search(line):
                prefix_match = log_prefix_pattern.search(line)
                timestamp = float(prefix_match.group(1))
                soar_decision_records.append({'timestamp': timestamp, 'counter': None})

    soar_decision_df = pd.DataFrame(soar_decision_records)
    return sender_msgs, receiver_msgs, soar_decision_df

def main():
    parser = argparse.ArgumentParser(description='Evaluate MIMO sender/receiver log files.')
    parser.add_argument('-d', '--run-dir', type=str, required=True, help='Directory containing log files')
    args = parser.parse_args()

    log_dir = os.path.join(args.run_dir, "logs")
    data_dir = os.path.join(args.run_dir, "data")
    eval_dir = os.path.join(args.run_dir, "evaluation")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(eval_dir, exist_ok=True)

    log_pattern = r'.*f_(\d+)\.log'
    print(os.listdir(log_dir))
    log_files = [f for f in os.listdir(log_dir) if re.match(log_pattern, f)]

    if log_files == []:
        log_message = f'No log files found in {log_dir} matching pattern.'
        raise Exception(log_message)

    message_results = []
    soar_decision_dfs = []

    for log_file in log_files:
        freq_match = re.search(r'f_(\d+\.?\d*)', log_file)
        frequency = float(freq_match.group(1)) if freq_match else None

        sender_msgs, receiver_msgs, soar_decision_df = parse_log_file(os.path.join(log_dir, log_file))

        # Save Soar decision cycle DataFrame to a file if any
        if not soar_decision_df.empty:
            soar_decision_df["frequency"] = frequency
            soar_decision_dfs.append(soar_decision_df)

        if receiver_msgs == []:
            print(f'No receiver messages found in log file {log_file}, skipping.')
            continue

        df = pd.DataFrame(receiver_msgs)


        # Map sender time based on frame_id and derive input_index from frame_id
        def get_sender_time(row):
            # Match sender time using counter and channel columns
            entry = sender_msgs.get((row['counter'], row['channel']), None)
            if isinstance(entry, dict):
                return entry.get('sender_time', None)
            return entry

        df['sender_time'] = df.apply(get_sender_time, axis=1)
        df['duration'] = df['receive_time'] - df['sender_time']
        df['frequency'] = frequency
        message_results.append(df)

    # Aggregate and plot duration vs frequency
    if message_results:
        message_results = pd.concat(message_results)
        message_results.to_csv(os.path.join(data_dir, f'combined_message_results.csv'), index=False)
    
    if soar_decision_dfs:
        soar_decision_dfs = pd.concat(soar_decision_dfs)
        soar_decision_dfs.to_csv(os.path.join(data_dir, f'combined_soar_decision.csv'), index=False)

    print(f'Parsing logs completed! Results saved to {data_dir}')

if __name__ == '__main__':
    main()
