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

def add_sliding_mean(ax, x, y, window=100, color="black", linewidth=2.5):
    x = np.asarray(x)
    y = np.asarray(y)

    smooth = pd.Series(y).rolling(
        window=window,
        center=True,
        min_periods=window//2
    ).mean()

    ax.plot(
        x,
        smooth,
        color=color,
        linewidth=linewidth
    )


def plot_metric(data, key, ylabel, title, total_color="black", window=100):

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

        # raw signal (transparent)
        axes[1].plot(
            d["t"],
            d[key],
            color=colors[i],
            alpha=0.25,
            label=name
        )

        # sliding mean (strong)
        add_sliding_mean(
            axes[1],
            d["t"],
            d[key],
            window=window,
            color=colors[i],
            linewidth=2
        )

    # total raw
    axes[0].plot(
        t,
        total,
        color=total_color,
        alpha=1.0
    )

    # total mean
    add_sliding_mean(
        axes[0],
        t,
        total,
        window=window,
        color="black",
        linewidth=3
    )

    axes[0].set_ylabel(ylabel)
    axes[0].set_title("Total")
    axes[0].grid(True)

    axes[1].set_ylabel(ylabel)
    axes[1].set_xlabel("time (s)")
    axes[1].set_title("Individual nodes")
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()
