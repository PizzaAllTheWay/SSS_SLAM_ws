import os
import numpy as np
from utils import (
    get_newest_file,
    load_csv,
    create_stacked_plot,
    add_series,
    add_sliding_mean,
    finalize_plot
)

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

def main():

    perf_file = get_newest_file(DATA_DIR, "performance")
    df = load_csv(perf_file)

    # relative time
    t0 = df["t"].iloc[0]
    df["t_rel"] = df["t"] - t0

    cut = 2.0
    df = df[(df["t_rel"] > cut) & (df["t_rel"] < df["t_rel"].iloc[-1] - cut)]
    df = df.reset_index(drop=True)

    runtime = df["runtime_s"].values * 1e3
    cpu = df["cpu_percent"].values
    ram = df["ram_mb"].values

    fig, axes = create_stacked_plot(
        3,
        title="State Estimator Performance",
        xlabel="Time [s]",
        ylabels=["Runtime [ms]", "CPU [% of core]", "RAM [MB]"]
    )

    add_series(axes[0], df["t_rel"], runtime, label="Callback runtime", color="blue")
    add_sliding_mean(axes[0], df["t_rel"], runtime, window=100, label="Runtime mean")

    add_series(axes[1], df["t_rel"], cpu, label="CPU usage", color="red")
    add_sliding_mean(axes[1], df["t_rel"], cpu, window=100, label="CPU mean")

    add_series(axes[2], df["t_rel"], ram, label="RAM usage", color="green")
    add_sliding_mean(axes[2], df["t_rel"], ram, window=100, label="RAM mean")

    finalize_plot()

if __name__ == "__main__":
    main()
