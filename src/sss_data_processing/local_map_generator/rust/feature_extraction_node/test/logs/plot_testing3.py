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

    # Keep only meaningful intensity range.
    # Remove invalid/background values like 0 and 255.
    mask = (image >= vmin) & (image <= vmax) & (image != 0) & (image != 255)
    out[mask] = image[mask]

    return out


def make_red_blue_overlay(bright_mask, shadow_mask):
    overlay = np.zeros((bright_mask.shape[0], bright_mask.shape[1], 3), dtype=np.uint8)

    # Blue = shadow candidates
    overlay[shadow_mask > 0] = [0, 0, 255]

    # Red = bright candidates
    overlay[bright_mask > 0] = [255, 0, 0]

    return overlay


def make_purple_overlay(mask):
    overlay = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)

    # Purple = post-morphology semantic mask
    overlay[mask > 0] = [255, 0, 255]

    return overlay


def threshold_bright_candidates_local(image, window_size=31, offset=1):
    image_f = image.astype(np.float32)

    valid_mask = (image != 0) & (image != 255)
    valid_f = valid_mask.astype(np.float32)

    # Local mean intensity from nearby valid pixels only.
    local_sum = ndi.uniform_filter(image_f * valid_f, size=window_size, mode="nearest")
    local_count = ndi.uniform_filter(valid_f, size=window_size, mode="nearest")

    local_mean = np.zeros_like(image_f, dtype=np.float32)
    np.divide(local_sum, local_count, out=local_mean, where=local_count > 0)

    # Bright pixel = above local mean by some offset.
    mask = valid_mask & (image_f >= (local_mean + offset))

    out = np.zeros_like(image, dtype=np.uint8)
    out[mask] = 255

    return out


def threshold_shadow_candidates_local(image, window_size=31, offset=1):
    image_f = image.astype(np.float32)

    valid_mask = (image != 0) & (image != 255)
    valid_f = valid_mask.astype(np.float32)

    # Local mean intensity from nearby valid pixels only.
    local_sum = ndi.uniform_filter(image_f * valid_f, size=window_size, mode="nearest")
    local_count = ndi.uniform_filter(valid_f, size=window_size, mode="nearest")

    local_mean = np.zeros_like(image_f, dtype=np.float32)
    np.divide(local_sum, local_count, out=local_mean, where=local_count > 0)

    # Shadow pixel = below local mean by some offset.
    mask = valid_mask & (image_f <= (local_mean - offset))

    out = np.zeros_like(image, dtype=np.uint8)
    out[mask] = 255

    return out


def filter_semantic_pairs(
    bright_mask,
    shadow_mask,
    search_radius=12,
    min_support=8,
):
    bright_bin = bright_mask > 0
    shadow_bin = shadow_mask > 0

    # We look in a local neighborhood around each bright/shadow pixel.
    # If a bright pixel has enough nearby shadow support, it survives.
    # If a shadow pixel has enough nearby bright support, it survives.
    kernel_size = 2 * search_radius + 1
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    shadow_support = ndi.convolve(shadow_bin.astype(np.uint8), kernel, mode="constant", cval=0)
    bright_support = ndi.convolve(bright_bin.astype(np.uint8), kernel, mode="constant", cval=0)

    bright_paired = bright_bin & (shadow_support >= min_support)
    shadow_paired = shadow_bin & (bright_support >= min_support)

    # Final semantic candidate mask is union of surviving bright and shadow pixels.
    paired_mask = np.zeros_like(bright_mask, dtype=np.uint8)
    paired_mask[bright_paired | shadow_paired] = 255

    return (
        (bright_paired.astype(np.uint8) * 255),
        (shadow_paired.astype(np.uint8) * 255),
        paired_mask,
    )


def apply_morphology_sequence(mask, operations):
    work = mask > 0
    structure = np.ones((3, 3), dtype=bool)

    # Apply a sequence like:
    # [("dilate", 2), ("erode", 1), ("close", 2)]
    # so you can tune shape growth/shrinking step by step.
    for op_name, iterations in operations:
        if iterations <= 0:
            continue

        if op_name == "dilate":
            work = ndi.binary_dilation(work, structure=structure, iterations=iterations)
        elif op_name == "erode":
            work = ndi.binary_erosion(work, structure=structure, iterations=iterations)
        elif op_name == "open":
            work = ndi.binary_opening(work, structure=structure, iterations=iterations)
        elif op_name == "close":
            work = ndi.binary_closing(work, structure=structure, iterations=iterations)
        else:
            raise ValueError(f"Unknown morphology operation: {op_name}")

    out = np.zeros_like(mask, dtype=np.uint8)
    out[work] = 255

    return out


def simple_semantic_threshold(image, relevant_min=1, local_window_size=51, local_offset=3):
    relevant = threshold_relevant_range(image, vmin=relevant_min, vmax=254)

    # Window of local values.
    # Bigger window = more global behavior and less noise.
    # Smaller window = more local behavior and more sensitivity to tiny local changes.
    # Offset is how far above/below the local mean a pixel must be to count.
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

    # Raw local semantic overlay.
    overlay = make_red_blue_overlay(bright, shadow)

    # New semantic step:
    # Keep only bright/shadow pixels that have nearby support from the opposite class.
    # This removes isolated red or blue speckles that do not participate in a local bright-shadow pattern.
    bright_paired, shadow_paired, paired_mask = filter_semantic_pairs(
        bright,
        shadow,
        search_radius=21,
        min_support=100,
    )

    # Now make a Dialation method an then removing and the dialoation and stuff like that dialate erode function where we give in the paired_mask
    # and then we do a certain amount of erosoan or doations folowed by a certain amount of erosion or dialions and so forth and so foth an array basicaly and get a image back
    # Then we use PURPLE color an dthsi is another mask that we will then overlay over the image jesjes do that please :)
    # Alos plot the overlay over the map now the Purple one notthe green one, also purple should have ist own plot as well :)

    # Paired red/blue overlay after pruning.
    paired_overlay = make_red_blue_overlay(bright_paired, shadow_paired)

    # Morphology step after semantic pairing.
    # Tune this sequence however you want.
    morphology_operations = [
        ("dilate", 4),
        ("erode", 1),
        ("close", 2),
        ("dilate", 4),
        ("erode", 2),
        ("close", 4),
    ]

    purple_mask = apply_morphology_sequence(
        paired_mask,
        morphology_operations,
    )

    # Final purple semantic mask / overlay.
    purple_overlay = make_purple_overlay(purple_mask)

    return {
        "relevant": relevant,
        "bright": bright,
        "shadow": shadow,
        "overlay": overlay,
        "bright_paired": bright_paired,
        "shadow_paired": shadow_paired,
        "paired_mask": paired_mask,
        "paired_overlay": paired_overlay,
        "purple_mask": purple_mask,
        "purple_overlay": purple_overlay,
    }


def plot_semantic_pipeline(df, title="Relevant / Bright / Shadow / Semantic"):
    if len(df) == 0:
        raise ValueError("No samples to plot")

    idx0 = 0

    cmap_img = cm.get_cmap("copper").copy()
    cmap_img.set_bad(color="white")

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
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

    ax0, ax1, ax2, ax3, ax4, ax5 = axes.ravel()

    im0 = ax0.imshow(masked(image), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
    ax0.set_title("Filtered")

    im1 = ax1.imshow(masked(result["relevant"]), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
    ax1.set_title("Relevant")

    im2 = ax2.imshow(result["overlay"], interpolation="nearest")
    ax2.set_title("Raw Bright=Red, Shadow=Blue")

    im3 = ax3.imshow(result["paired_overlay"], interpolation="nearest")
    ax3.set_title("Paired Bright=Red, Shadow=Blue")

    im4 = ax4.imshow(result["purple_overlay"], interpolation="nearest")
    ax4.set_title("Purple Post-Morphology Mask")

    ax5.imshow(masked(image), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
    ax5.imshow(result["purple_overlay"], interpolation="nearest", alpha=0.65)
    ax5.set_title("Purple Overlay on Filtered")

    for ax in [ax0, ax1, ax2, ax3, ax4, ax5]:
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
        im3.set_data(result["paired_overlay"])
        im4.set_data(result["purple_overlay"])

        ax5.clear()
        ax5.imshow(masked(image), cmap=cmap_img, interpolation="nearest", vmin=vmin, vmax=vmax)
        ax5.imshow(result["purple_overlay"], interpolation="nearest", alpha=0.65)
        ax5.set_title("Purple Overlay on Filtered")
        ax5.set_xlabel("x [pixel]")
        ax5.set_ylabel("y [pixel]")

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
        title="Relevant / Bright / Shadow / Semantic",
    )