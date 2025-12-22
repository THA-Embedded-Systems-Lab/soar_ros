import os
import re
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def parse_log_file(filepath):
    sender_pattern = re.compile(r'Published header to input(\d+): frame_id=([^,]+), sec=(\d+), nanosec=(\d+)')
    receiver_pattern = re.compile(r'Received header on output(\d+): frame_id=([^,]+), sec=(\d+), nanosec=(\d+)')
    receive_time_pattern = re.compile(r'receive_time=(\d+\.\d+)')

    sender_msgs = {}  # {(frame_id, input_index): timestamp}
    receiver_msgs = []

    with open(filepath, 'r') as f:
        for line in f:
            sender_match = sender_pattern.search(line)
            if sender_match:
                input_index = int(sender_match.group(1))
                frame_id = sender_match.group(2)
                sec = int(sender_match.group(3))
                nanosec = int(sender_match.group(4))
                sender_msgs[(frame_id, input_index)] = sec + nanosec * 1e-9
                
            receiver_match = receiver_pattern.search(line)
            if receiver_match:
                output_index = int(receiver_match.group(1))
                frame_id = receiver_match.group(2)
                sec = int(receiver_match.group(3))
                nanosec = int(receiver_match.group(4))
                receive_time_match = receive_time_pattern.search(line)
                if receive_time_match:
                    receive_time = float(receive_time_match.group(1))
                else:
                    receive_time = sec + nanosec * 1e-9
                receiver_msgs.append({
                    'frame_id': frame_id,
                    'output_index': output_index,
                    'receive_time': receive_time
                })
                
    return sender_msgs, receiver_msgs

def main():
    parser = argparse.ArgumentParser(description='Evaluate MIMO sender/receiver log files.')
    parser.add_argument('--log_dir', type=str, required=True, help='Directory containing log files')
    parser.add_argument('--run_id', type=str, default=None, help='Run identifier to filter log files')
    args = parser.parse_args()

    log_pattern = r'.*mimo.*f_(\d+\.?\d*)_id_(\w+)\.log'
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
        
        # Map sender time based on frame_id and derive input_index from frame_id
        def get_sender_time(row):
            # frame_id format is "counter_index" (e.g., "0_0", "0_1", "0_2")
            parts = row['frame_id'].split('_')
            if len(parts) == 2:
                input_index = int(parts[1])
                return sender_msgs.get((row['frame_id'], input_index), None)
            return None
            
        df['sender_time'] = df.apply(get_sender_time, axis=1)
        df['duration'] = df['receive_time'] - df['sender_time']
        df['frequency'] = frequency
        df['run_id'] = run_id
        results.append(df)

        # Analyze dropped messages per output
        for output_idx in df['output_index'].unique():
            output_df = df[df['output_index'] == output_idx].sort_values('receive_time')
            dropped = []
            frame_ids = output_df['frame_id'].values
            
            # Extract counter from frame_id (e.g., "0_1" -> 0)
            counters = [int(fid.split('_')[0]) for fid in frame_ids]
            for i in range(1, len(counters)):
                if counters[i] != counters[i-1] + 1:
                    dropped.append((counters[i-1], counters[i]))
            print(f'Run {run_id}, Frequency {frequency} Hz, Output {output_idx}: Dropped messages: {dropped}')

        # Message order derivative per output
        for output_idx in df['output_index'].unique():
            output_df = df[df['output_index'] == output_idx].sort_values('receive_time').reset_index(drop=True)
            counters = [int(fid.split('_')[0]) for fid in output_df['frame_id'].values]
            derivative = pd.Series(counters).diff().fillna(0)
            
            plt.figure()
            plt.plot(output_df.index, derivative, marker='x')
            plt.title(f'Frame ID Derivative Output{output_idx} (Run {run_id}, f={frequency} Hz)')
            plt.xlabel('Message Index')
            plt.ylabel('frame_id derivative')
            plt.grid()
            plt.savefig(os.path.join(args.log_dir, f'frameid_derivative_{run_id}_f{frequency}_output{output_idx}.png'))
            plt.close()

    # Aggregate and plot duration vs frequency
    if results:
        all_df = pd.concat(results)
        
        # Extract counter from frame_id for plotting
        all_df['counter'] = all_df['frame_id'].apply(lambda x: int(x.split('_')[0]))
        
        # Plot per output
        for output_idx in sorted(all_df['output_index'].unique()):
            plt.figure()
            output_df = all_df[all_df['output_index'] == output_idx]
            for freq in sorted(output_df['frequency'].unique()):
                sub = output_df[output_df['frequency'] == freq]
                plt.plot(sub['counter'], sub['duration'], label=f'f={freq} Hz', alpha=0.7)
            plt.title(f'Duration between sender and receiver (Output {output_idx})')
            plt.xlabel('Frame Counter')
            plt.ylabel('Duration (s)')
            plt.legend()
            plt.grid()
            plt.savefig(os.path.join(args.log_dir, f'duration_vs_frameid_output{output_idx}.png'))
            plt.close()

        # Overall duration plot (all outputs)
        plt.figure()
        for freq in sorted(all_df['frequency'].unique()):
            sub = all_df[all_df['frequency'] == freq]
            plt.plot(sub['counter'], sub['duration'], '.', label=f'f={freq} Hz', alpha=0.5)
        plt.title('Duration between sender and receiver (All Outputs)')
        plt.xlabel('Frame Counter')
        plt.ylabel('Duration (s)')
        plt.legend()
        plt.grid()
        plt.savefig(os.path.join(args.log_dir, 'duration_vs_frameid_all.png'))
        plt.close()

        # New plot: receive time vs duration (normalized so each frequency starts at t=0)
        for output_idx in sorted(all_df['output_index'].unique()):
            plt.figure()
            output_df = all_df[all_df['output_index'] == output_idx]
            for freq in sorted(output_df['frequency'].unique()):
                sub = output_df[output_df['frequency'] == freq].copy()
                sub['norm_receive_time'] = sub['receive_time'] - sub['receive_time'].iloc[0]
                plt.plot(sub['norm_receive_time'], sub['duration'], label=f'f={freq} Hz', alpha=0.7)
            plt.title(f'Duration vs Receive Time (Output {output_idx}, normalized t=0)')
            plt.xlabel('Normalized Receive Time (s)')
            plt.ylabel('Duration (s)')
            plt.legend()
            plt.grid()
            plt.savefig(os.path.join(args.log_dir, f'duration_vs_time_output{output_idx}.png'))
            plt.close()

        print(f'\nEvaluation complete! Results saved to {args.log_dir}')

if __name__ == '__main__':
    main()
