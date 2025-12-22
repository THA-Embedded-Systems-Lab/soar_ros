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
                channel = int(sender_match.group(1))
                counter = int(sender_match.group(2).split("_")[0])
                sec = int(sender_match.group(3))
                nanosec = int(sender_match.group(4))
                sender_msgs[(counter, channel)] = sec + nanosec * 1e-9
                
            receiver_match = receiver_pattern.search(line)
            if receiver_match:
                channel = int(receiver_match.group(1))
                counter = int(receiver_match.group(2).split("_")[0])
                sec = int(receiver_match.group(3))
                nanosec = int(receiver_match.group(4))
                receive_time_match = receive_time_pattern.search(line)
                if receive_time_match == None:
                    raise "No receive time found via regex."
                receive_time = float(receive_time_match.group(1))
                receiver_msgs.append({
                    'counter': counter,
                    'channel': channel,
                    'receive_time': receive_time
                })
                
    return sender_msgs, receiver_msgs

def main():
    parser = argparse.ArgumentParser(description='Evaluate MIMO sender/receiver log files.')
    parser.add_argument('-d', '--run-dir', type=str, required=True, help='Directory containing log files')
    args = parser.parse_args()

    log_dir = os.path.join(args.run_dir, "logs")
    data_dir = os.path.join(args.run_dir, "data")
    eval_dir = os.path.join(args.run_dir, "evaluation")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(eval_dir, exist_ok=True)

    run_id = os.path.basename(os.path.normpath(args.run_dir))

    log_pattern = r'.*mimo.*f_(\d+\.?\d*)_id_([\d\-T:+]+)\.log'
    log_files = [f for f in os.listdir(log_dir) if re.match(log_pattern, f)]
    if run_id:
        log_files = [f for f in log_files if f'id_{run_id}.log' in f]
    
    if log_files == []:
        raise Exception('No log files found matching the criteria.')
        
    results = []
    for log_file in log_files:
        freq_match = re.search(r'f_(\d+\.?\d*)', log_file)
        runid_match = re.search(r'id_([\d\-T:+]+)', log_file)
        frequency = float(freq_match.group(1)) if freq_match else None
        run_id = runid_match.group(1) if runid_match else None
        
        sender_msgs, receiver_msgs = parse_log_file(os.path.join(log_dir, log_file))
        df = pd.DataFrame(receiver_msgs)
        
        # Map sender time based on frame_id and derive input_index from frame_id
        def get_sender_time(row):
            # Match sender time using counter and channel columns
            return sender_msgs.get((row['counter'], row['channel']), None)
            
        df['sender_time'] = df.apply(get_sender_time, axis=1)
        df['duration'] = df['receive_time'] - df['sender_time']
        df['frequency'] = frequency
        df['run_id'] = run_id
        results.append(df)

        df.to_csv(os.path.join(data_dir, f'results_{run_id}_f{frequency}.csv'), index=False)

        # Analyze dropped messages per output
        for channel in df['channel'].unique():
            output_df = df[df['channel'] == channel].sort_values('receive_time')
            dropped = []
            counters = output_df['counter'].values
            
            for i in range(1, len(counters)):
                if counters[i] != counters[i-1] + 1:
                    dropped.append((counters[i-1], counters[i]))
            print(f'Run {run_id}, Frequency {frequency} Hz, Output {channel}: Dropped messages: {dropped}')

        # Message order derivative per output
        for channel in df['channel'].unique():
            output_df = df[df['channel'] == channel].sort_values('receive_time').reset_index(drop=True)
            counters = output_df['counter'].values
            derivative = pd.Series(counters).diff().fillna(0)
            
            plt.figure()
            plt.plot(output_df.index, derivative, marker='x')
            plt.title(f'Frame ID Derivative Output{channel} (Run {run_id}, f={frequency} Hz)')
            plt.xlabel('Message Index')
            plt.ylabel('frame_id derivative')
            plt.grid()
            plt.savefig(os.path.join(eval_dir, f'frameid_derivative_{run_id}_f{frequency}_output{channel}.png'))
            plt.close()

    # Aggregate and plot duration vs frequency
    if results:
        all_df = pd.concat(results)
        all_df.to_csv(os.path.join(data_dir, f'all_results.csv'), index=False)

        
        print(f'\nEvaluation complete! Results saved to {eval_dir}')

if __name__ == '__main__':
    main()
