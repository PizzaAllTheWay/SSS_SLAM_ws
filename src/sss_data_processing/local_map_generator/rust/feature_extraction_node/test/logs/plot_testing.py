import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.widgets import Slider
from scipy import ndimage as ndi

from utils import (
    get_newest_file,
    load_csv,
    get_processed_image_from_row,
)


def threshold_relevant_range(image, vmin=70, vmax=254):
    out = np.zeros_like(image, dtype=np.uint8)
    mask = (image >= vmin) & (image <= vmax) & (image != 0) & (image != 255)
    out[mask] = image[mask]
    return out

def make_red_blue_overlay(bright_mask, shadow_mask):
    overlay = np.zeros((bright_mask.shape[0], bright_mask.shape[1], 3), dtype=np.uint8)
    overlay[shadow_mask > 0] = [0, 0, 255]
    overlay[bright_mask > 0] = [255, 0, 0]
    return overlay

def threshold_bright_candidates_local(image, window_size=31, offset=1):
    image_f = image.astype(np.float32)

    valid_mask = (image != 0) & (image != 255)
    valid_f = valid_mask.astype(np.float32)

    local_sum = ndi.uniform_filter(image_f * valid_f, size=window_size, mode="nearest")
    local_count = ndi.uniform_filter(valid_f, size=window_size, mode="nearest")

    local_mean = np.zeros_like(image_f, dtype=np.float32)
    np.divide(local_sum, local_count, out=local_mean, where=local_count > 0)

    mask = valid_mask & (image_f >= (local_mean + offset))

    out = np.zeros_like(image, dtype=np.uint8)
    out[mask] = 255
    return out


def threshold_shadow_candidates_local(image, window_size=31, offset=1):
    image_f = image.astype(np.float32)

    valid_mask = (image != 0) & (image != 255)
    valid_f = valid_mask.astype(np.float32)

    local_sum = ndi.uniform_filter(image_f * valid_f, size=window_size, mode="nearest")
    local_count = ndi.uniform_filter(valid_f, size=window_size, mode="nearest")

    local_mean = np.zeros_like(image_f, dtype=np.float32)
    np.divide(local_sum, local_count, out=local_mean, where=local_count > 0)

    mask = valid_mask & (image_f <= (local_mean - offset))

    out = np.zeros_like(image, dtype=np.uint8)
    out[mask] = 255
    return out


def simple_semantic_threshold(image, relevant_min=50, local_window_size=51, local_offset=3):
    relevant = threshold_relevant_range(image, vmin=relevant_min, vmax=254)

    # window of local values, the bigger the window size the more global it becomes and less noisy
    # However to much and global semantic takes over witch is bad as valyes and rocks have their own local smeantics
    # To small and the local sematics become to strong an dbecomes a lot more noisy
    # Keep somewheer in the midle
    # Offest is what pixel is considered included form the local mean
    # Usueally a very small offset is recomended, however it depends on the windows size, if the window is small havig local offset to big will never give you any candidate pixels
    # Meanwhile if teh window size big and local offset to small it will h\give to many unwanted candiates
    # However form experiance offset = 1 works well all things form wndow size 11 to widow size 51, so honwetly just keep it like that
    bright = threshold_bright_candidates_local(
        relevant,
        window_size=local_window_size,
        offset=local_offset,
    )

    shadow = threshold_shadow_candidates_local(
        relevant,
        window_size=local_window_size,
        offset=local_offset,
    )

    overlay = make_red_blue_overlay(bright, shadow)

    return {
        "relevant": relevant,
        "bright": bright,
        "shadow": shadow,
        "overlay": overlay,
    }


def plot_semantic_pipeline(df, title="Relevant / Bright / Shadow"):
    if len(df) == 0:
        raise ValueError("No samples to plot")

    idx0 = 0

    cmap_img = cm.get_cmap("copper").copy()
    cmap_img.set_bad(color="white")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    plt.subplots_adjust(bottom=0.16)

    def masked(img):
        return np.ma.masked_where(img == 0, img)

    def compute(idx):
        image = get_processed_image_from_row(df.iloc[idx])
        result = simple_semantic_threshold(image)
        return image, result

    image, result = compute(idx0)

    valid = image[(image != 0) & (image != 255)]
    vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
    vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

    ax0, ax1, ax2, ax3 = axes.ravel()

    im0 = ax0.imshow(masked(image), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
    ax0.set_title("Filtered")

    im1 = ax1.imshow(masked(result["relevant"]), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
    ax1.set_title("Relevant")

    im2 = ax2.imshow(result["overlay"], interpolation="nearest")
    ax2.set_title("Bright=Red, Shadow=Blue")

    im3 = ax3.imshow(masked(image), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
    ax3.imshow(result["overlay"], interpolation="nearest", alpha=0.65)
    ax3.set_title("Overlay on Filtered")

    for ax in [ax0, ax1, ax2, ax3]:
        ax.set_xlabel("x [pixel]")
        ax.set_ylabel("y [pixel]")

    slider_ax = fig.add_axes([0.15, 0.05, 0.7, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Sample",
        valmin=0,
        valmax=len(df) - 1,
        valinit=idx0,
        valstep=1,
    )

    def update(val):
        idx = int(slider.val)
        image, result = compute(idx)

        valid = image[(image != 0) & (image != 255)]
        vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
        vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

        im0.set_data(masked(image))
        im1.set_data(masked(result["relevant"]))
        im2.set_data(result["overlay"])

        ax3.clear()
        ax3.imshow(masked(image), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
        ax3.imshow(result["overlay"], interpolation="nearest", alpha=0.65)
        ax3.set_title("Overlay on Filtered")
        ax3.set_xlabel("x [pixel]")
        ax3.set_ylabel("y [pixel]")

        im0.set_clim(vmin=vmin, vmax=vmax)
        im1.set_clim(vmin=vmin, vmax=vmax)

        fig.suptitle(title)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    fig.suptitle(title)
    plt.show()


if __name__ == "__main__":
    BASE_DIR = os.path.dirname(__file__)
    LOG_DATA_DIR = os.path.join(BASE_DIR, "data")

    filtered_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "filtered_image"),
    )

    plot_semantic_pipeline(
        filtered_df,
        title="Relevant / Bright / Shadow",
    )