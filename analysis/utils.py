import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.abspath(__file__))

def resolve_dataset_paths(datasets):
    resolved = {}
    for name, rel_path in datasets.items():
        resolved[name] = os.path.abspath(os.path.join(ROOT, "..", rel_path))
    return resolved

def load_perf(path):
    df = pd.read_csv(path)

    t0 = df["t"].iloc[0]
    df["t_rel"] = df["t"] - t0

    return df

def resample_perf(df, window):
    bins = np.arange(df["t_rel"].min(), df["t_rel"].max(), window)

    df = df.copy()
    df["bin"] = np.digitize(df["t_rel"], bins)

    g = df.groupby("bin").mean(numeric_only=True)

    return {
        "t": bins[:len(g)],
        "cpu": g["cpu_percent"].values,
        "ram": g["ram_mb"].values,
        "runtime": g["runtime_s"].values * 1000
    }

def load_all_perf(datasets, window):
    data = {}
    paths = resolve_dataset_paths(datasets)

    for name, path in paths.items():
        df = load_perf(path)
        data[name] = resample_perf(df, window)

    return data

def plot_metric(data, key, ylabel, title, total_color="black"):

    fig, axes = plt.subplots(2, 1, sharex=True)
    fig.suptitle(title)

    total = None
    t = None

    colors = plt.cm.tab10(np.linspace(0, 1, len(data)))

    for i, (name, d) in enumerate(data.items()):

        if total is None:
            total = d[key].copy()
            t = d["t"]
        else:
            total += d[key]

        axes[1].plot(
            d["t"],
            d[key],
            label=name,
            color=colors[i]
        )

    axes[0].plot(
        t,
        total,
        linewidth=3,
        color=total_color,
        label="Total"
    )

    axes[0].set_ylabel(ylabel)
    axes[0].set_title("Total")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].set_ylabel(ylabel)
    axes[1].set_xlabel("time (s)")
    axes[1].set_title("Individual nodes")
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()
