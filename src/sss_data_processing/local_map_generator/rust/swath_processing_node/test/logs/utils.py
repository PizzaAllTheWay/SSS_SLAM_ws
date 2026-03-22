import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm



# FILE HELPERS ----------
def get_newest_file(directory, prefix=None):
    pattern = "*.csv" if prefix is None else f"{prefix}_*.csv"
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError("No matching csv files found")
    return max(files, key=os.path.getctime)


def load_csv(path, start_sample=None, end_sample=None):
    df = pd.read_csv(path)

    if start_sample is not None or end_sample is not None:
        df = df.iloc[start_sample:end_sample].reset_index(drop=True)

    return df

# SWATH PLOT ----------
def parse_array(col, target_len=None):
    rows = [list(map(float, s.split())) for s in col]

    if target_len is None:
        target_len = max(len(r) for r in rows)

    out = np.zeros((len(rows), target_len))
    for i, r in enumerate(rows):
        out[i, :len(r)] = r[:target_len]

    return out

def plot_swath(
    df,
    title="Swath",
    ylabel="Ping #",
    xlabel="Across track",
    zero_color="white",
    cmap_name="copper",
):
    port = parse_array(df["port"])
    stb = parse_array(df["starboard"])
    swath = np.hstack([port, stb])

    swath_masked = np.ma.masked_where(swath == 0, swath)

    cmap = cm.get_cmap(cmap_name).copy()
    cmap.set_bad(color=zero_color)

    valid = swath[swath != 0]
    vmin = np.percentile(valid, 0.05)
    vmax = np.percentile(valid, 99)

    plt.figure()
    plt.imshow(
        swath_masked,
        aspect="auto",
        cmap=cmap,
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
    )
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.tight_layout()
    plt.show()