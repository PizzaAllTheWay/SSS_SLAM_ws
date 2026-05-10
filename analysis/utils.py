import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# HELPERS ----------
ROOT = os.path.dirname(os.path.abspath(__file__))

def resolve_dataset_paths(datasets):
    resolved = {}
    for name, rel_path in datasets.items():
        resolved[name] = os.path.abspath(os.path.join(ROOT, "..", rel_path))
    return resolved

# PERFORMANCE ----------
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

def get_distinct_colors(n, seed=42):
    rng = np.random.default_rng(seed)
    colors = plt.cm.hsv(np.linspace(0, 1, n, endpoint=False))
    rng.shuffle(colors)
    return colors

def plot_metric(data, key, ylabel, title, total_color="black", window=100):

    fig, axes = plt.subplots(2, 1, sharex=True)
    fig.suptitle(title)

    total = None
    t = None

    colors = get_distinct_colors(len(data))

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

# SCHEDULABILITY ----------
def load_schedulability_from_df(df, t_start, t_stop, median_samples=20):
    df = df.copy().sort_values("t").reset_index(drop=True)
    df = df[(df["t"] >= t_start) & (df["t"] <= t_stop)].copy()

    if len(df) < 2:
        return {
            "t": np.array([0.0]),
            "utilization": np.array([0.0]),
            "period_s": np.array([0.0]),
        }

    # Local period between activations of THIS logical task.
    df["period_s"] = df["t"].diff()

    # First sample has no previous period, so use the next available period.
    df.loc[df.index[0], "period_s"] = df["t"].iloc[1] - df["t"].iloc[0]
    df = df[df["period_s"] > 0.0].copy()

    # Adaptive T_i: rolling median of this task's own activation period.
    df["period_median_s"] = df["period_s"].rolling(
        window=median_samples,
        center=True,
        min_periods=1,
    ).median()

    # Utilization: U = C / T_i.
    df["utilization"] = df["runtime_s"] / df["period_median_s"]
    df["t_rel"] = df["t"] - t_start

    return {
        "t": df["t_rel"].to_numpy(),
        "utilization": df["utilization"].to_numpy(),
        "period_s": df["period_median_s"].to_numpy(),
    }


def load_schedulability(path, t_start, t_stop, median_samples=20):
    df = pd.read_csv(path)
    return load_schedulability_from_df(
        df,
        t_start,
        t_stop,
        median_samples=median_samples,
    )


def split_runtime_categories(
    df,
    factor=2.0,
    low_name="fast_task",
    high_name="slow_task",
):
    df = df.copy().sort_values("t").reset_index(drop=True)

    median_runtime = df["runtime_s"].median()
    threshold = median_runtime * factor

    low_df = df[df["runtime_s"] <= threshold].copy()
    high_df = df[df["runtime_s"] > threshold].copy()

    return {
        low_name: low_df,
        high_name: high_df,
    }


def load_split_schedulability(
    path,
    t_start,
    t_stop,
    median_samples=20,
    factor=2.0,
    low_name="fast_task",
    high_name="slow_task",
):
    df = pd.read_csv(path)

    split_dataframes = split_runtime_categories(
        df,
        factor=factor,
        low_name=low_name,
        high_name=high_name,
    )

    data = {}

    for name, split_df in split_dataframes.items():
        data[name] = load_schedulability_from_df(
            split_df,
            t_start,
            t_stop,
            median_samples=median_samples,
        )

    return data


def load_all_schedulability(
    datasets,
    t_start,
    t_stop,
    median_samples=20,
    split_datasets=None,
):
    data = {}
    paths = resolve_dataset_paths(datasets)

    if split_datasets is None:
        split_datasets = {}

    for name, path in paths.items():
        if name in split_datasets:
            cfg = split_datasets[name]

            split_data = load_split_schedulability(
                path,
                t_start,
                t_stop,
                median_samples=median_samples,
                factor=cfg.get("factor", 2.0),
                low_name=cfg.get("low_name", f"{name}_fast"),
                high_name=cfg.get("high_name", f"{name}_slow"),
            )

            data.update(split_data)
        else:
            data[name] = load_schedulability(
                path,
                t_start,
                t_stop,
                median_samples=median_samples,
            )

    return data


def build_total_utilization(data):
    # Use all real event timestamps. No fixed scheduler time window.
    all_t = np.unique(np.concatenate([d["t"] for d in data.values()]))
    total = np.zeros_like(all_t)

    for d in data.values():
        u = np.interp(
            all_t,
            d["t"],
            d["utilization"],
            left=0.0,
            right=0.0,
        )
        total += u

    return all_t, total


def plot_schedulability(data, title="Schedulability", smooth_window=100):
    fig, axes = plt.subplots(2, 1, sharex=True)
    fig.suptitle(title)

    colors = get_distinct_colors(len(data))

    t_total, total = build_total_utilization(data)

    axes[0].plot(
        t_total,
        total,
        color="black",
        alpha=0.35,
        label="total raw",
    )

    add_sliding_mean(
        axes[0],
        t_total,
        total,
        window=smooth_window,
        color="black",
        linewidth=3,
    )

    for i, (name, d) in enumerate(data.items()):
        axes[1].plot(
            d["t"],
            d["utilization"],
            color=colors[i],
            alpha=0.25,
            label=name,
        )

        add_sliding_mean(
            axes[1],
            d["t"],
            d["utilization"],
            window=smooth_window,
            color=colors[i],
            linewidth=2,
        )

    axes[0].axhline(1.0, color="red", linestyle="--", linewidth=1.5, label="U = 1")
    axes[1].axhline(1.0, color="red", linestyle="--", linewidth=1.0)

    axes[0].set_ylabel("Utilization")
    axes[0].set_title("Total")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    axes[1].set_ylabel("Utilization")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title("Individual nodes")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()
