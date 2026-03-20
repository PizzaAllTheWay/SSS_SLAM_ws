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

# SWATH PLOT ----------
def parse_array(col, target_len=None):
    rows = [list(map(float, s.split())) for s in col]

    if target_len is None:
        target_len = max(len(r) for r in rows)  # keep full data

    out = np.zeros((len(rows), target_len))
    for i, r in enumerate(rows):
        out[i, :len(r)] = r[:target_len]

    return out

def plot_swath(raw_df, proc_df, title="Swath (Raw vs Processed)", y_max=None):
    # RAW → keep full resolution
    raw_port = parse_array(raw_df["port"])
    raw_stb  = parse_array(raw_df["starboard"])
    raw = np.hstack([raw_port, raw_stb])  # flip PORT sides

    # PROCESSED → same width as raw
    target_len = raw_port.shape[1]
    proc_port = parse_array(proc_df["port"], target_len)
    proc_stb  = parse_array(proc_df["starboard"], target_len)
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
