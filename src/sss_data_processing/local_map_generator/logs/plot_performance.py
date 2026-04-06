import os
import numpy as np

from utils import (
    get_newest_file,
    load_csv,
    build_performance_timeline,
    create_stacked_plot,
    add_series,
    add_sliding_mean,
    add_dropout_spans,
    finalize_plot,
)

def main():
    # Swath Processing (START) --------------------------------------------------
    DATA_DIR = os.path.join(os.path.dirname(__file__), "data/swath")
    perf_file = get_newest_file(DATA_DIR, "performance")
    df = load_csv(perf_file)

    # Set these from your actual experiment window
    t_start = 1756381027.016462
    t_stop  = 1756382637.907318

    # Fallback if you want quick testing from file only
    if t_start == 0.0 and t_stop == 0.0:
        t_start = df["t"].iloc[0]
        t_stop = df["t"].iloc[-1]

    gap_threshold = 5.0

    df, dropout_spans, zero_dt = build_performance_timeline(
        df=df,
        t_start=t_start,
        t_stop=t_stop,
        gap_threshold=gap_threshold,
        zero_dt=None,
    )

    runtime_ms = df["runtime_s"].values * 1e3
    cpu = df["cpu_percent"].values
    ram = df["ram_mb"].values

    fig, axes = create_stacked_plot(
        3,
        title="Swath Processing Performance",
        xlabel="Time since t_start [s]",
        ylabels=["Runtime [ms]", "CPU [%]", "RAM [MB]"]
    )

    add_series(axes[0], df["t_rel"], runtime_ms, label="Callback runtime", color="blue")
    add_sliding_mean(axes[0], df["t_rel"], runtime_ms, window=100, color="black", label="Runtime mean")
    add_dropout_spans(axes[0], dropout_spans, t_ref=t_start)

    add_series(axes[1], df["t_rel"], cpu, label="CPU usage", color="red")
    add_sliding_mean(axes[1], df["t_rel"], cpu, window=100, color="black", label="CPU mean")
    add_dropout_spans(axes[1], dropout_spans, t_ref=t_start)

    add_series(axes[2], df["t_rel"], ram, label="RAM usage", color="green")
    add_sliding_mean(axes[2], df["t_rel"], ram, window=100, color="black", label="RAM mean")
    add_dropout_spans(axes[2], dropout_spans, t_ref=t_start)

    finalize_plot()
    # Swath Processing (STOP) --------------------------------------------------

    # Map Generation (START) --------------------------------------------------
    DATA_DIR = os.path.join(os.path.dirname(__file__), "data/map")
    perf_file = get_newest_file(DATA_DIR, "performance")
    df = load_csv(perf_file)

    # Set these from your actual experiment window
    t_start = 1756381027.016462
    t_stop  = 1756382637.907318

    # Fallback if you want quick testing from file only
    if t_start == 0.0 and t_stop == 0.0:
        t_start = df["t"].iloc[0]
        t_stop = df["t"].iloc[-1]

    gap_threshold = 5.0

    df, dropout_spans, zero_dt = build_performance_timeline(
        df=df,
        t_start=t_start,
        t_stop=t_stop,
        gap_threshold=gap_threshold,
        zero_dt=None,
    )

    runtime_ms = df["runtime_s"].values * 1e3
    cpu = df["cpu_percent"].values
    ram = df["ram_mb"].values

    fig, axes = create_stacked_plot(
        3,
        title="Swath Processing Performance",
        xlabel="Time since t_start [s]",
        ylabels=["Runtime [ms]", "CPU [%]", "RAM [MB]"]
    )

    add_series(axes[0], df["t_rel"], runtime_ms, label="Callback runtime", color="blue")
    add_sliding_mean(axes[0], df["t_rel"], runtime_ms, window=100, color="black", label="Runtime mean")
    add_dropout_spans(axes[0], dropout_spans, t_ref=t_start)

    add_series(axes[1], df["t_rel"], cpu, label="CPU usage", color="red")
    add_sliding_mean(axes[1], df["t_rel"], cpu, window=100, color="black", label="CPU mean")
    add_dropout_spans(axes[1], dropout_spans, t_ref=t_start)

    add_series(axes[2], df["t_rel"], ram, label="RAM usage", color="green")
    add_sliding_mean(axes[2], df["t_rel"], ram, window=100, color="black", label="RAM mean")
    add_dropout_spans(axes[2], dropout_spans, t_ref=t_start)

    finalize_plot()
    # Map Generation (STOP) --------------------------------------------------

if __name__ == "__main__":
    main()