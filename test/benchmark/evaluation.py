import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

def build_paths(session_id: str) -> dict:
    return {
        "no_load":              f"{session_id}_ch0_msg0_f1000_delTrue_sortFalse",
        "siso":                 f"{session_id}_ch1_msg4000_f2000-2500-3000_delTrue_sortFalse",
        "normal":               f"{session_id}_ch1_msg8000_f200_delFalse_sortFalse",
        "auto_delete_io":       f"{session_id}_ch1_msg8000_f200_delTrue_sortFalse",
        "benchmark_preferences": f"{session_id}_ch1_msg200_f2000-2500_delTrue_sortTrue",
    }

def save_latex_table(df, filepath, caption="", label="", float_format="%.4f"):
    """Save a DataFrame as a LaTeX booktabs table file."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    latex = df.to_latex(
        index=False,
        float_format=float_format,
        escape=False,
        column_format="l" + "r" * (len(df.columns) - 1),
        caption=caption,
        label=label,
    )
    with open(filepath, "w") as f:
        f.write(latex)
    print(f"Saved LaTeX table: {filepath}")


# Distribution plot of decision cycle frequencies (No Load) using matplotlib
# Compute histograms and normalize to 100%
def compare_soar_kernel_frequencies(data_frame, scenario: str, legend = True):
    """
    Plot distribution of Soar decision cycle frequencies.
    Creates a single plot with all unique frequencies overlaid.
    Exports mean/SD statistics as a LaTeX table with a filename matching the plot.

    Parameters:
    -----------
    data_frame : pandas.DataFrame
        Soar decision cycle dataframe with 'frequency' and 'timestamp' columns
    scenario : str
        Scenario name for saving the plot (e.g., "no_load", "siso", "mimo")
    """
    # Get all unique frequencies from the dataframe
    unique_frequencies = sorted(data_frame["frequency"].unique())

    plt.figure(figsize=(3.3, 3))
    plt.ylabel("Count (%)")
    plt.xlabel("Frequency (Hz)")

    ax = plt.gca()  # Get current axes
    ax.ticklabel_format(style='scientific', axis='x', scilimits=(0,0))

    stats_rows = []

    for frequency in unique_frequencies:
        df = data_frame[data_frame["frequency"] == frequency]

        if df.empty:
            print(f"Skipping frequency {frequency} Hz - no data available")
            continue

        freqs = 1 / df["timestamp"].diff().dropna()
        # Filter out infinite and NaN values
        freqs = freqs[np.isfinite(freqs)]

        if len(freqs) == 0:
            print(f"Skipping frequency {frequency} Hz - no valid frequency data")
            continue

        hist, bins = np.histogram(freqs, bins=50)
        hist_pct = hist / hist.sum() * 100
        bin_centers = (bins[:-1] + bins[1:]) / 2

        plt.bar(bin_centers, hist_pct, width=(bins[1]-bins[0]), alpha=0.5, label=f"{frequency} Hz")

        mean_val = freqs.mean()
        std_val = freqs.std()
        print(f"{scenario} {frequency} Hz: Mean = {mean_val:.2f} Hz, SD = {std_val:.2f} Hz")

        stats_rows.append({
            "Frequency (Hz)": int(frequency),
            "Mean (Hz)": round(mean_val, 2),
            "Std Dev (Hz)": round(std_val, 2),
            "N": len(freqs),
        })

    if legend:
        plt.legend()
    plt.xlim(left=0)

    if not os.path.exists("out/images"):
        os.makedirs("out/images")
    base_name = f"kernel_frequency_comparison_{scenario}"
    plt.tight_layout()
    plt.savefig(f"out/images/{base_name}.png", dpi=300)

    plt.title(f"Distribution of Soar Decision Cycle Frequencies - {scenario}")
    # plt.show()

    if stats_rows:
        save_latex_table(
            pd.DataFrame(stats_rows),
            f"out/images/{base_name}.tex",
            caption=f"Soar kernel decision cycle frequency statistics -- {scenario}",
            label=f"tab:kernel_freq_{scenario}",
            float_format="%.2f",
        )


def analyze_sender_frequency(df, show_plots=False, scenario=""):
    rows = []
    for freq in df["frequency"].unique():
        df_subset = df[(df["frequency"] == freq)]

        for channel in df_subset["channel"].unique():
            df_channel = df_subset[df_subset["channel"] == channel].sort_values("counter")
            # Reset the index so x-axis starts from 0 for each new plot
            mean = (1/df_channel["sender_time"].diff()).mean()
            sd = (1/df_channel["sender_time"].diff()).std()
            print(f"Sender: F {freq} Channel {channel} M: {mean:0.6} Hz, SD: {sd:0.6} Hz")
            rows.append({
                "Frequency (Hz)": int(freq),
                "Channel": int(channel),
                "Mean (Hz)": round(mean, 4),
                "Std Dev (Hz)": round(sd, 4),
            })
    if rows:
        os.makedirs("out/images", exist_ok=True)
        tag = f"_{scenario}" if scenario else ""
        save_latex_table(
            pd.DataFrame(rows),
            f"out/images/sender_frequency_stats{tag}.tex",
            caption=f"Sender frequency statistics{' -- ' + scenario if scenario else ''}",
            label=f"tab:sender_freq{'_' + scenario if scenario else ''}",
            float_format="%.4f",
        )
    return pd.DataFrame(rows) if rows else pd.DataFrame()

def plot_dual_axis_comparison(test_data, soar_data, title_prefix="", figsize=(6.8, 2), min_duration=None):
    """
    Plot test data duration and Soar kernel frequency on aligned dual y-axes.
    Creates a separate plot for each unique frequency in test_data.
    Exports per-frequency duration statistics as a LaTeX table alongside the plots.

    Parameters:
    -----------
    test_data : pandas.DataFrame
        Test results dataframe with 'frequency', 'receive_time', and 'duration' columns
    soar_data : pandas.DataFrame
        Soar decision cycle dataframe with 'frequency' and 'timestamp' columns
    title_prefix : str
        Prefix for the plot title (e.g., "SISO", "MIMO")
    figsize : tuple
        Figure size (width, height)
    min_duration : float or None
        Minimum x-axis duration in seconds. If set, the x-axis will span at least this
        many seconds, useful for comparing plots across scenarios on the same scale.

    Returns:
    --------
    float
        The maximum x-axis duration (in seconds) across all plotted frequencies,
        useful for passing as min_duration when comparing another scenario.
    """
    # Get all unique frequencies from test_data
    unique_frequencies = sorted(test_data["frequency"].unique())

    max_duration = 0.0
    dur_rows = []

    for frequency in unique_frequencies:
        fig, ax1 = plt.subplots(figsize=figsize)

        # Data preparation
        df_test = test_data[test_data["frequency"] == frequency]
        df_soar = soar_data[soar_data["frequency"] == frequency]

        if df_test.empty or df_soar.empty:
            print(f"Skipping frequency {frequency} Hz - no data available")
            plt.close(fig)
            continue

        global_start = min(df_test["receive_time"].min(), df_soar["timestamp"].min())

        rel_receive_time = df_test["receive_time"] - global_start
        rel_timestamp = df_soar["timestamp"] - global_start
        freqs = 1 / df_soar["timestamp"].diff().dropna()

        min_time = min(rel_receive_time.min(), rel_timestamp.min())
        max_time = max(rel_receive_time.max(), rel_timestamp.max())

        if min_duration is not None:
            max_time = max(max_time, min_time + min_duration)

        max_duration = max(max_duration, max_time - min_time)

        # Left y-axis: Duration
        # color1 = 'tab:blue'
        ax1.set_xlabel("Test duration (s)")
        ax1.set_ylabel("Message delay (s)")

        for channel in df_test["channel"].unique():
            df_channel = df_test[df_test["channel"] == channel]
            ax1.plot(rel_receive_time[df_channel.index], df_channel["duration"], marker='o', linestyle='', markersize=2, label=f"Duration Channel {channel}", zorder=2)
            dur = df_channel["duration"]
            dur_rows.append({
                "Frequency (Hz)": int(frequency),
                "Channel": int(channel),
                "Mean (s)": round(dur.mean(), 6),
                "Std Dev (s)": round(dur.std(), 6),
                "Median (s)": round(dur.median(), 6),
                "Min (s)": round(dur.min(), 6),
                "Max (s)": round(dur.max(), 6),
                "P95 (s)": round(dur.quantile(0.95), 6),
                "N": len(dur),
            })

        ax1.tick_params(axis='y')
        ax1.set_xlim(min_time, max_time)
        ax1.set_zorder(2)
        ax1.patch.set_visible(False)

        ax2 = ax1.twinx()
        ax2.set_zorder(1)
        color2 = 'tab:green'
        ax2.set_ylabel("$f$ Kernel (Hz)", color=color2)
        ax2.set_yscale('log')
        ax2.fill_between(rel_timestamp[1:], freqs, 0, color=color2, alpha=0.2, zorder=0, linestyle="")
        ax2.tick_params(axis='y', labelcolor=color2)

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        # ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

        title = f"{title_prefix} {int(frequency)} Hz Duration and Soar Kernel Frequency vs Relative Time"
        ax2.grid(False)

        fig.tight_layout()

        if not os.path.exists("out/images"):
            os.makedirs("out/images")
        plt.savefig(f"out/images/{title.replace(' ', '_')}.png", dpi=300)

        fig.suptitle(title)
        # plt.show()

    if dur_rows and title_prefix:
        tag = title_prefix.replace(" ", "_")
        save_latex_table(
            pd.DataFrame(dur_rows),
            f"out/images/duration_stats_{tag}.tex",
            caption=f"Message delay statistics -- {title_prefix}",
            label=f"tab:duration_{tag.lower()}",
            float_format="%.6f",
        )

    return max_duration

def trim_soar_timeseries(soar_df, duration_df, soar_time_col="timestamp", duration_time_col="receive_time", frequency_col="frequency", min_duration=0.0):
    trimmed_frames = []
    for freq in sorted(duration_df[frequency_col].unique()):
        duration_subset = duration_df[duration_df[frequency_col] == freq]
        soar_subset = soar_df[soar_df[frequency_col] == freq]
        if duration_subset.empty or soar_subset.empty:
            continue
        min_time = duration_subset[duration_time_col].min() - 2.0
        max_time = max(
            duration_subset[duration_time_col].max() + 2.0,
            duration_subset[duration_time_col].min() + min_duration + 2.0
        )
        trimmed = soar_subset[(soar_subset[soar_time_col] >= min_time) & (soar_subset[soar_time_col] <= max_time)].copy()
        print(
            f"Trimmed {freq} Hz Soar timeseries to range: {min_time:.3f} - {max_time:.3f} "
            f"(original: {soar_subset[soar_time_col].min():.3f} - {soar_subset[soar_time_col].max():.3f})"
        )
        trimmed_frames.append(trimmed)
    if not trimmed_frames:
        return soar_df.iloc[0:0].copy()
    return pd.concat(trimmed_frames, ignore_index=True)

def load_benchmark_data(dir_name_logs, min_duration=0.0) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Load and process benchmark data from a specified directory.

    This function loads the benchmark test results and Soar decision cycle data from CSV files
    in the specified directory, trims the Soar timeseries to match the test data time range,
    and returns the processed DataFrames.

    Parameters:
    -----------
    dir_name_logs : str
        The directory name containing the benchmark logs (e.g., "2026-02-11T10:12:30_CHANNELS_1").
        The function expects the directory to be located under "out/<dir_name_logs>/data/".
    min_duration : float
        Minimum duration (in seconds) to retain in the trimmed Soar timeseries beyond the
        test data start. Use this to ensure Soar data is available when comparing with a
        longer scenario via min_duration in plot_dual_axis_comparison.

    Returns:
    --------
    tuple[pd.DataFrame, pd.DataFrame]
        A tuple containing:
        - df_test : pd.DataFrame
            The test message results DataFrame loaded from "combined_message_results.csv".
            Contains columns like 'frequency', 'receive_time', 'duration', 'channel', etc.
        - df_soar_trimmed : pd.DataFrame
            The trimmed Soar decision cycle DataFrame loaded from "combined_soar_decision.csv"
            and trimmed to the time range of the test data using trim_soar_timeseries().
            Contains columns like 'frequency', 'timestamp', etc.

    Raises:
    -------
    FileNotFoundError
        If the expected CSV files are not found in the specified directory.
    """
    base_path = os.path.join("out", dir_name_logs,"data")
    messages_log = os.path.join(base_path,"combined_message_results.csv")

    if os.path.exists(messages_log):
        df_test = pd.read_csv(messages_log)
    else:
        df_test = pd.DataFrame()
    df_soar = pd.read_csv(os.path.join(base_path,"combined_soar_decision.csv"))
    if os.path.exists(messages_log):
        df_soar = trim_soar_timeseries(df_soar, df_test, min_duration=min_duration)
    return df_test, df_soar

def frequency_comparison_plot(df,title_prefix):
    plt.figure(figsize=(6.8, 2))
    plt.xlabel('Message Counter (1)')
    plt.ylabel('Message delay (s)')

    stat_rows = []
    for frequencies in sorted(df['frequency'].unique()):
        freq_df = df[df['frequency'] == frequencies]
        for channel in freq_df['channel'].unique():
            df_channel = freq_df[freq_df['channel'] == channel].sort_values(by='counter')
            plt.plot(df_channel['counter'], df_channel['duration'], marker='o', linestyle='', label=f'{frequencies} Hz',markersize=2)
            dur = df_channel['duration']
            stat_rows.append({
                "Frequency (Hz)": int(frequencies),
                "Channel": int(channel),
                "Mean (s)": round(dur.mean(), 6),
                "Std Dev (s)": round(dur.std(), 6),
                "Median (s)": round(dur.median(), 6),
                "Min (s)": round(dur.min(), 6),
                "Max (s)": round(dur.max(), 6),
                "P95 (s)": round(dur.quantile(0.95), 6),
                "N": len(dur),
            })

    title = f"{title_prefix} Input Frequency Comparison"
    plt.legend()

    if not os.path.exists("out/images"):
        os.makedirs("out/images")

    base_name = title.replace(' ', '_')
    plt.tight_layout()
    plt.savefig(f"out/images/{base_name}.png", dpi=300)

    plt.title(title)
    # plt.show()

    if stat_rows:
        save_latex_table(
            pd.DataFrame(stat_rows),
            f"out/images/{base_name}.tex",
            caption=f"Message delay statistics per frequency -- {title_prefix}",
            label=f"tab:freq_comparison_{title_prefix.lower().replace(' ', '_')}",
            float_format="%.6f",
        )


def main(paths: dict):
    matplotlib.use("Agg")
    plt.rcParams['axes.grid'] = True

    df_messages_no_load, df_no_load = load_benchmark_data(paths["no_load"])
    compare_soar_kernel_frequencies(df_no_load, "no_load", False)

    df_siso, df_siso_soar_trimmed = load_benchmark_data(paths["siso"])
    analyze_sender_frequency(df_siso, scenario="siso")
    compare_soar_kernel_frequencies(df_siso_soar_trimmed, "siso")
    frequency_comparison_plot(df_siso, "SISO")
    plot_dual_axis_comparison(df_siso, df_siso_soar_trimmed, "SISO")

    df_normal, df_normal_soar_trimmed = load_benchmark_data(paths["normal"])
    normal_test_duration = df_normal["receive_time"].max() - df_normal["receive_time"].min()
    df_auto_delete_io, df_auto_delete_io_soar_trimmed = load_benchmark_data(paths["auto_delete_io"], min_duration=normal_test_duration)

    analyze_sender_frequency(df_normal, scenario="no_autodelete")
    normal_duration = plot_dual_axis_comparison(df_normal, df_normal_soar_trimmed, "no_autodelete")
    analyze_sender_frequency(df_auto_delete_io, scenario="auto_delete")
    plot_dual_axis_comparison(df_auto_delete_io, df_auto_delete_io_soar_trimmed, "auto_delete", min_duration=normal_duration)

    df_msg_preferences, df_soar_preferences = load_benchmark_data(paths["benchmark_preferences"])
    analyze_sender_frequency(df_msg_preferences, scenario="preferences")
    plot_dual_axis_comparison(df_msg_preferences, df_soar_preferences, "Preference including")
    frequency_comparison_plot(df_msg_preferences, "preference_enabled")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate benchmark results for a given session.")
    parser.add_argument("session_id", help="SESSION_ID printed at the end of benchmark.sh (e.g. 2026-06-13T17:28:51)")
    args = parser.parse_args()
    main(build_paths(args.session_id))
