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

def load_perf(path, t_start, t_stop, t_delta=5.0):
    df = pd.read_csv(path)
    df = df.copy().sort_values("t").reset_index(drop=True)

    rows = []
    last_t = t_start

    for _, row in df.iterrows():
        t = row["t"]

        if t < t_start or t > t_stop:
            continue

        if t - last_t > t_delta:
            rows.append({
                "t": last_t,
                "cpu_percent": 0.0,
                "ram_mb": 0.0,
                "runtime_s": 0.0,
            })
            rows.append({
                "t": t,
                "cpu_percent": 0.0,
                "ram_mb": 0.0,
                "runtime_s": 0.0,
            })

        rows.append({
            "t": t,
            "cpu_percent": row["cpu_percent"],
            "ram_mb": row["ram_mb"],
            "runtime_s": row["runtime_s"],
        })

        last_t = t

    if len(rows) == 0:
        rows = [{
            "t": t_start,
            "cpu_percent": 0.0,
            "ram_mb": 0.0,
            "runtime_s": 0.0,
        }]

    if t_stop - last_t > t_delta:
        rows.append({
            "t": last_t,
            "cpu_percent": 0.0,
            "ram_mb": 0.0,
            "runtime_s": 0.0,
        })
        rows.append({
            "t": t_stop,
            "cpu_percent": 0.0,
            "ram_mb": 0.0,
            "runtime_s": 0.0,
        })

    df = pd.DataFrame(rows)
    df["t_rel"] = df["t"] - t_start
    return df


def resample_perf(df, window, t_start, t_stop):
    t_rel_stop = t_stop - t_start
    bins = np.arange(0.0, t_rel_stop + window, window)

    df = df.copy()
    df["bin"] = np.digitize(df["t_rel"], bins) - 1
    df = df[(df["bin"] >= 0) & (df["bin"] < len(bins) - 1)]

    g = df.groupby("bin").mean(numeric_only=True)

    cpu = np.zeros(len(bins) - 1)
    ram = np.zeros(len(bins) - 1)
    runtime = np.zeros(len(bins) - 1)

    idx = g.index.to_numpy(dtype=int)
    cpu[idx] = g["cpu_percent"].to_numpy()
    ram[idx] = g["ram_mb"].to_numpy()
    runtime[idx] = g["runtime_s"].to_numpy() * 1000.0

    return {
        "t": bins[:-1],
        "cpu": cpu,
        "ram": ram,
        "runtime": runtime
    }


def load_all_perf(datasets, window, t_start, t_stop, t_delta=5.0):
    data = {}
    paths = resolve_dataset_paths(datasets)

    for name, path in paths.items():
        df = load_perf(path, t_start, t_stop, t_delta)
        data[name] = resample_perf(df, window, t_start, t_stop)

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
