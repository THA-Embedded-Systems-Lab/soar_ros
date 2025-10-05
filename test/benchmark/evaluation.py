import os
import re
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def parse_log_file(filepath):
    sender_pattern = re.compile(r'Published header: frame_id=(\d+), sec=(\d+), nanosec=(\d+)')
    receiver_pattern = re.compile(r'Received header: frame_id=(\d+), sec=(\d+), nanosec=(\d+)')
    receive_time_pattern = re.compile(r'receive_time=(\d+\.\d+)')

    sender_msgs = {}
    receiver_msgs = []

    with open(filepath, 'r') as f:
        for line in f:
            sender_match = sender_pattern.search(line)
            if sender_match:
                frame_id = int(sender_match.group(1))
                sec = int(sender_match.group(2))
                nanosec = int(sender_match.group(3))
                sender_msgs[frame_id] = sec + nanosec * 1e-9
            receiver_match = receiver_pattern.search(line)
            if receiver_match:
                frame_id = int(receiver_match.group(1))
                sec = int(receiver_match.group(2))
                nanosec = int(receiver_match.group(3))
                receive_time_match = receive_time_pattern.search(line)
                if receive_time_match:
                    receive_time = float(receive_time_match.group(1))
                else:
                    receive_time = sec + nanosec * 1e-9
                receiver_msgs.append({'frame_id': frame_id, 'receive_time': receive_time})
    return sender_msgs, receiver_msgs

def main():
    parser = argparse.ArgumentParser(description='Evaluate sender/receiver log files.')
    parser.add_argument('--log_dir', type=str, required=True, help='Directory containing log files')
    parser.add_argument('--run_id', type=str, default=None, help='Run identifier to filter log files')
    args = parser.parse_args()

    log_pattern = r'.*f_(\d+\.?\d*)_id_(\w+)\.log'
    log_files = [f for f in os.listdir(args.log_dir) if re.match(log_pattern, f)]
    if args.run_id:
        log_files = [f for f in log_files if f'id_{args.run_id}.log' in f]
    results = []
    for log_file in log_files:
        freq_match = re.search(r'f_(\d+\.?\d*)', log_file)
        runid_match = re.search(r'id_(\w+)', log_file)
        frequency = float(freq_match.group(1)) if freq_match else None
        run_id = runid_match.group(1) if runid_match else None
        sender_msgs, receiver_msgs = parse_log_file(os.path.join(args.log_dir, log_file))
        df = pd.DataFrame(receiver_msgs)
        df['sender_time'] = df['frame_id'].map(sender_msgs)
        df['duration'] = df['receive_time'] - df['sender_time']
        df['frequency'] = frequency
        df['run_id'] = run_id
        results.append(df)

        # Dropped messages
        dropped = []
        frame_ids = df['frame_id'].values
        for i in range(1, len(frame_ids)):
            if frame_ids[i] != frame_ids[i-1] + 1:
                dropped.append((frame_ids[i-1], frame_ids[i]))
        print(f'Run {run_id}, Frequency {frequency} Hz: Dropped messages: {dropped}')

        # Message order derivative
        derivative = pd.Series(frame_ids).diff().fillna(0)
        plt.figure()
        plt.plot(df.index, derivative, marker='x')
        plt.title(f'Frame ID Derivative (Run {run_id}, f={frequency} Hz)')
        plt.xlabel('Message Index')
        plt.ylabel('frame_id derivative')
        plt.grid()
        plt.savefig(os.path.join(args.log_dir, f'frameid_derivative_{run_id}_f{frequency}.png'))

    # Aggregate and plot duration vs frequency
    if results:
        all_df = pd.concat(results)
        plt.figure()
        for freq in sorted(all_df['frequency'].unique()):
            sub = all_df[all_df['frequency'] == freq]
            plt.plot(sub['frame_id'], sub['duration'], label=f'f={freq} Hz')
        plt.title('Duration between sender and receiver')
        plt.xlabel('Frame ID')
        plt.ylabel('Duration (s)')
        plt.legend()
        plt.grid()
        plt.savefig(os.path.join(args.log_dir, 'duration_vs_frameid.png'))

        # New plot: receive time vs duration (normalized so each frequency starts at t=0)
        plt.figure()
        for freq in sorted(all_df['frequency'].unique()):
            sub = all_df[all_df['frequency'] == freq].copy()
            sub['norm_receive_time'] = sub['receive_time'] - sub['receive_time'].iloc[0]
            plt.plot(sub['norm_receive_time'], sub['duration'], label=f'f={freq} Hz')
        plt.title('Duration between sender and receiver (x=receive_time, y=duration, t=0 per freq)')
        plt.xlabel('Normalized Receive Time (s)')
        plt.ylabel('Duration (s)')
        plt.legend()
        plt.grid()
        plt.savefig(os.path.join(args.log_dir, 'duration_vs_time.png'))
        plt.show()

        # Compute actual frequency based on sender timestamps
        for freq in sorted(all_df['frequency'].unique()):
            sub = all_df[all_df['frequency'] == freq].copy()
            sender_times = sub['sender_time'].dropna().values
            if len(sender_times) > 1:
                intervals = sender_times[1:] - sender_times[:-1]
                actual_freq = 1.0 / intervals.mean()
                print(f'Expected frequency: {freq} Hz, Actual sender frequency: {actual_freq:.3f} Hz')
            else:
                print(f'Expected frequency: {freq} Hz, Actual sender frequency: N/A (not enough data)')

if __name__ == '__main__':
    main()
