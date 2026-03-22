import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2

# FILE HELPERS ----------
def get_newest_file(directory, prefix=None):
    pattern = "*.csv" if prefix is None else f"{prefix}_*.csv"
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError("No matching csv files found")
    return max(files, key=os.path.getctime)

def load_csv(path):
    return pd.read_csv(path)

# TIME / DROPOUT HELPERS ----------
def build_performance_timeline(
    df,
    t_start,
    t_stop,
    gap_threshold=5.0,
    zero_dt=None,
):
    df = df.copy().sort_values("t").reset_index(drop=True)
    df = df[(df["t"] >= t_start) & (df["t"] <= t_stop)].reset_index(drop=True)

    if df.empty:
        raise ValueError("No performance samples inside [t_start, t_stop].")

    if zero_dt is None:
        dt = np.diff(df["t"].values)
        dt_valid = dt[dt > 0]
        zero_dt = np.median(dt_valid) if len(dt_valid) > 0 else 0.1

    rows = []
    dropout_spans = []

    first_t = df["t"].iloc[0]
    if first_t - t_start > gap_threshold:
        dropout_spans.append((t_start, first_t))

        n_fill = int(np.floor((first_t - t_start) / zero_dt))
        if n_fill > 0:
            fill_times = t_start + zero_dt * np.arange(0, n_fill + 1)

            for tf in fill_times:
                if tf >= first_t:
                    break
                rows.append({
                    "t": tf,
                    "runtime_s": 0.0,
                    "cpu_percent": 0.0,
                    "ram_mb": 0.0,
                    "is_artificial": True,
                })

    for i in range(len(df) - 1):
        row = df.iloc[i].to_dict()
        row["is_artificial"] = False
        rows.append(row)

        t0 = df.iloc[i]["t"]
        t1 = df.iloc[i + 1]["t"]
        dt_gap = t1 - t0

        if dt_gap > gap_threshold:
            dropout_spans.append((t0, t1))

            n_fill = int(np.floor(dt_gap / zero_dt)) - 1
            if n_fill > 0:
                fill_times = t0 + zero_dt * np.arange(1, n_fill + 1)

                for tf in fill_times:
                    if tf >= t1:
                        break
                    rows.append({
                        "t": tf,
                        "runtime_s": 0.0,
                        "cpu_percent": 0.0,
                        "ram_mb": 0.0,
                        "is_artificial": True,
                    })

    last_row = df.iloc[-1].to_dict()
    last_row["is_artificial"] = False
    rows.append(last_row)

    last_t = df["t"].iloc[-1]
    if t_stop - last_t > gap_threshold:
        dropout_spans.append((last_t, t_stop))

        n_fill = int(np.floor((t_stop - last_t) / zero_dt))
        if n_fill > 0:
            fill_times = last_t + zero_dt * np.arange(1, n_fill + 1)

            for tf in fill_times:
                if tf > t_stop:
                    break
                rows.append({
                    "t": tf,
                    "runtime_s": 0.0,
                    "cpu_percent": 0.0,
                    "ram_mb": 0.0,
                    "is_artificial": True,
                })

    out = pd.DataFrame(rows)
    out = out.sort_values("t").reset_index(drop=True)
    out["t_rel"] = out["t"] - t_start

    return out, dropout_spans, zero_dt

# PLOT HELPERS ----------
def finalize_plot():
    for ax in plt.gcf().axes:
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()

def create_stacked_plot(n_rows, title="Title", xlabel="Time", ylabels=None, sharex=True):
    fig, axes = plt.subplots(n_rows, 1, sharex=sharex)
    fig.suptitle(title)

    if n_rows == 1:
        axes = [axes]

    for i, ax in enumerate(axes):
        if ylabels:
            ax.set_ylabel(ylabels[i])
        ax.grid(True)

    axes[-1].set_xlabel(xlabel)
    return fig, axes

def add_series(ax, x, y, label=None, color=None, linestyle="-", linewidth=1.5, cov=None, sigma=2.0, alpha=0.2):
    ax.plot(
        x,
        y,
        label=label,
        color=color,
        linestyle=linestyle,
        linewidth=linewidth
    )

    if cov is not None:
        std = np.sqrt(cov)
        upper = y + sigma * std
        lower = y - sigma * std

        ax.fill_between(
            x,
            lower,
            upper,
            color=color,
            alpha=alpha
        )

def add_sliding_mean(ax, x, y, window=100, color="black", linewidth=2.5, label="Sliding mean"):
    x = np.asarray(x)
    y = np.asarray(y)

    smooth = pd.Series(y).rolling(
        window=window,
        center=True,
        min_periods=max(1, window // 2)
    ).mean()

    ax.plot(
        x,
        smooth,
        color=color,
        linewidth=linewidth,
        label=label
    )

def add_dropout_spans(ax, spans, t_ref=0.0, color="gray", alpha=0.25, label="Side Scan Sonar dropout"):
    label_added = False

    for t0, t1 in spans:
        x0 = t0 - t_ref
        x1 = t1 - t_ref

        if not label_added:
            ax.axvspan(x0, x1, color=color, alpha=alpha, label=label)
            label_added = True
        else:
            ax.axvspan(x0, x1, color=color, alpha=alpha)

# SWATH PLOT ----------
def parse_array(col, target_len=None):
    rows = []

    for v in col:
        # missing / NaN / bad cell -> empty row
        if pd.isna(v):
            rows.append([])
            continue

        # already numeric scalar -> treat as empty row
        if isinstance(v, (int, float, np.integer, np.floating)):
            rows.append([])
            continue

        s = str(v).strip()
        if not s:
            rows.append([])
            continue

        rows.append(list(map(float, s.split())))

    if target_len is None:
        target_len = max((len(r) for r in rows), default=0)

    out = np.zeros((len(rows), target_len))
    for i, r in enumerate(rows):
        n = min(len(r), target_len)
        if n > 0:
            out[i, :n] = r[:n]

    return out

def plot_swath(raw_df, proc_df, title="Swath (Raw vs Processed)", y_max=None):
    raw_port = parse_array(raw_df["port"])
    raw_stb = parse_array(raw_df["starboard"])
    raw = np.hstack([raw_port, raw_stb])

    target_len = raw_port.shape[1]
    proc_port = parse_array(proc_df["port"], target_len)
    proc_stb = parse_array(proc_df["starboard"], target_len)
    proc = np.hstack([proc_port, proc_stb])

    fig, axes = plt.subplots(2, 1, sharex=True, sharey=True)
    fig.suptitle(title)

    axes[0].imshow(raw, aspect="auto", cmap="copper")
    axes[0].set_title("Raw Swath")

    axes[1].imshow(proc, aspect="auto", cmap="copper")
    axes[1].set_title("Processed Swath")

    if y_max is not None:
        y_max = min(y_max, raw.shape[0])
        axes[0].set_ylim(y_max, 0)
        axes[1].set_ylim(y_max, 0)

    axes[1].set_xlabel("Across track")
    axes[0].set_ylabel("Ping #")
    axes[1].set_ylabel("Ping #")

    plt.tight_layout()
    plt.show()