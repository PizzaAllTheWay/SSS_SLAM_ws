# utils.py
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.widgets import Slider



# FILE HELPERS ----------
def get_newest_file(directory, prefix=None):
    pattern = "*.csv" if prefix is None else f"{prefix}_*.csv"
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError("No matching csv files found")
    return max(files, key=os.path.getctime)


def load_csv(path, start_sample=None, end_sample=None, start_t=None, end_t=None):
    df = pd.read_csv(path)

    if start_sample is not None or end_sample is not None:
        df = df.iloc[start_sample:end_sample]

    if start_t is not None:
        df = df[df["t"] >= start_t]

    if end_t is not None:
        df = df[df["t"] <= end_t]

    return df.reset_index(drop=True)


# PARSING ----------
def parse_flat_u8_image(image_str, width, height):
    data = np.fromstring(image_str, sep=" ", dtype=np.uint8)

    expected_len = width * height
    if data.size != expected_len:
        raise ValueError(
            f"Image size mismatch: got {data.size}, expected {expected_len}"
        )

    return data.reshape((height, width))


def get_map_image_from_row(row):
    width = int(row["width"])
    height = int(row["height"])
    try:
        image_str = row["map"]
    except:
        image_str = row["image"]
    return parse_flat_u8_image(image_str, width, height)


def get_processed_image_from_row(row):
    width = int(row["width"])
    height = int(row["height"])
    image_str = row["image"]
    return parse_flat_u8_image(image_str, width, height)


# PLOT ----------
def plot_before_after_image(before_df, after_df, title="Before / After"):
    n = min(len(before_df), len(after_df))
    if n == 0:
        raise ValueError("No samples to plot")

    before_df = before_df.iloc[:n].reset_index(drop=True)
    after_df = after_df.iloc[:n].reset_index(drop=True)

    idx0 = 0
    before_img = get_map_image_from_row(before_df.iloc[idx0])
    after_img = get_processed_image_from_row(after_df.iloc[idx0])

    raw_masked = np.ma.masked_where(before_img == 0, before_img)
    processed_masked = np.ma.masked_where(after_img == 0, after_img)

    cmap = cm.get_cmap("copper").copy()
    cmap.set_bad(color="white")

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), sharex=True, sharey=True)
    plt.subplots_adjust(bottom=0.18)

    ax_before, ax_after = axes

    raw_valid = before_img[before_img != 0]
    raw_vmin = np.percentile(raw_valid, 0.05) if len(raw_valid) > 0 else 0
    raw_vmax = np.percentile(raw_valid, 99) if len(raw_valid) > 0 else 1

    processed_valid = after_img[after_img != 0]
    processed_vmin = np.percentile(processed_valid, 0.05) if len(processed_valid) > 0 else 0
    processed_vmax = np.percentile(processed_valid, 99) if len(processed_valid) > 0 else 1

    im_before = ax_before.imshow(
        raw_masked,
        cmap=cmap,
        interpolation="nearest",
        vmin=raw_vmin,
        vmax=raw_vmax,
    )
    ax_before.set_title("Before")
    ax_before.set_xlabel("x [pixel]")
    ax_before.set_ylabel("y [pixel]")

    im_after = ax_after.imshow(
        processed_masked,
        cmap=cmap,
        interpolation="nearest",
        vmin=processed_vmin,
        vmax=processed_vmax,
    )
    ax_after.set_title("After")
    ax_after.set_xlabel("x [pixel]")
    ax_after.set_ylabel("y [pixel]")

    ax_after.sharex(ax_before)
    ax_after.sharey(ax_before)

    t0 = before_df.iloc[idx0]["t"] if "t" in before_df.columns else idx0
    fig.suptitle(f"{title}")

    slider_ax = fig.add_axes([0.15, 0.06, 0.7, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Sample",
        valmin=0,
        valmax=n - 1,
        valinit=idx0,
        valstep=1,
    )

    def update(val):
        idx = int(slider.val)

        before_img = get_map_image_from_row(before_df.iloc[idx])
        after_img = get_processed_image_from_row(after_df.iloc[idx])

        raw_masked = np.ma.masked_where(before_img == 0, before_img)
        processed_masked = np.ma.masked_where(after_img == 0, after_img)

        im_before.set_data(raw_masked)
        im_after.set_data(processed_masked)

        raw_valid = before_img[before_img != 0]
        if len(raw_valid) > 0:
            im_before.set_clim(
                vmin=float(np.percentile(raw_valid, 0.05)),
                vmax=float(np.percentile(raw_valid, 99)),
            )
        else:
            im_before.set_clim(vmin=0, vmax=1)

        processed_valid = after_img[after_img != 0]
        if len(processed_valid) > 0:
            im_after.set_clim(
                vmin=float(np.percentile(processed_valid, 0.05)),
                vmax=float(np.percentile(processed_valid, 99)),
            )
        else:
            im_after.set_clim(vmin=0, vmax=1)

        t = before_df.iloc[idx]["t"] if "t" in before_df.columns else idx
        fig.suptitle(f"{title}")

        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()