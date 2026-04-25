# utils.py
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.widgets import Slider
import matplotlib.patches as patches



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

def parse_mask_from_row(row):
    mask_width = int(row["mask_width"])
    mask_height = int(row["mask_height"])
    mask_str = row["mask"]
    return parse_flat_u8_image(mask_str, mask_width, mask_height)



# PLOT FOR Filter and Segmentation ----------
def plot_before_after_image(before_df, after_df, title="Before/After"):
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



# PLOT FOR Labelling ----------
def _darken_color(color, factor=0.55):
    color = np.array(color[:3], dtype=float)
    return tuple(np.clip(color * factor, 0.0, 1.0))

def plot_segmented_and_labeled(segmented_df, landmark_df, title="Before/After"):
    if len(segmented_df) == 0:
        raise ValueError("No segmented samples to plot")
    if len(landmark_df) == 0:
        raise ValueError("No landmark samples to plot")

    segmented_df = segmented_df.reset_index(drop=True)

    # Group landmark rows by timestamp.
    landmark_groups = {
        float(t): group.reset_index(drop=True)
        for t, group in landmark_df.groupby("t", sort=True)
    }

    # Only keep timestamps that exist in both dataframes.
    valid_indices = []
    for i in range(len(segmented_df)):
        t = float(segmented_df.iloc[i]["t"])
        if t in landmark_groups:
            valid_indices.append(i)

    if len(valid_indices) == 0:
        raise ValueError("No matching timestamps between segmented image csv and landmark csv")

    idx0 = 0

    cmap_img = cm.get_cmap("copper").copy()
    cmap_img.set_bad(color="white")

    cmap_labels = cm.get_cmap("tab20")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True)
    plt.subplots_adjust(bottom=0.16)

    ax_left, ax_right = axes

    def masked(img):
        return np.ma.masked_where(img == 0, img)

    def draw_sample(sample_idx):
        ax_left.clear()
        ax_right.clear()

        df_idx = valid_indices[sample_idx]
        seg_row = segmented_df.iloc[df_idx]
        t = float(seg_row["t"])

        segmented_image = get_processed_image_from_row(seg_row)
        landmark_rows = landmark_groups[t]

        valid = segmented_image[segmented_image != 0]
        vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
        vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

        # Left: raw segmented image
        ax_left.imshow(
            masked(segmented_image),
            cmap=cmap_img,
            interpolation="nearest",
            vmin=vmin,
            vmax=vmax,
        )
        ax_left.set_title("Segmented Image")
        ax_left.set_xlabel("x [pixel]")
        ax_left.set_ylabel("y [pixel]")

        # Right: labeled image with overlays
        ax_right.set_title("Labeled Landmarks")
        ax_right.set_xlabel("x [pixel]")
        ax_right.set_ylabel("y [pixel]")

        # Draw each landmark with its own color.
        for _, row in landmark_rows.iterrows():
            label_id = int(row["label_id"])
            cx = float(row["cx"])
            cy = float(row["cy"])
            x = int(row["bbox_x"])
            y = int(row["bbox_y"])
            width = int(row["bbox_width"])
            height = int(row["bbox_height"])
            area = int(row["area"])

            mask = parse_mask_from_row(row)

            color = cmap_labels(label_id % 20)
            dark_color = _darken_color(color)

            # Create colored RGBA mask overlay.
            rgba = np.zeros((height, width, 4), dtype=float)
            rgba[..., 0] = color[0]
            rgba[..., 1] = color[1]
            rgba[..., 2] = color[2]
            rgba[..., 3] = np.where(mask > 0, 0.35, 0.0)

            # Draw local mask at its bounding-box position.
            ax_right.imshow(
                rgba,
                interpolation="nearest",
                extent=(x, x + width, y + height, y),
            )

            # Draw bounding box.
            rect = patches.Rectangle(
                (x, y),
                width,
                height,
                linewidth=1.8,
                edgecolor=color,
                facecolor=(color[0], color[1], color[2], 0.10),
            )
            ax_right.add_patch(rect)

            # Draw centroid in darker color.
            ax_right.plot(
                cx,
                cy,
                marker="o",
                markersize=4,
                color=dark_color,
            )

            # Draw ID and area text near the box.
            ax_right.text(
                x,
                max(y - 3, 0),
                f"ID {label_id}, A={area}",
                color=color,
                fontsize=8,
                ha="left",
                va="bottom",
                bbox=dict(
                    facecolor=(1, 1, 1, 0.55),
                    edgecolor="none",
                    pad=1.5,
                ),
            )

        fig.suptitle(title)

    draw_sample(idx0)

    slider_ax = fig.add_axes([0.15, 0.06, 0.7, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Sample",
        valmin=0,
        valmax=len(valid_indices) - 1,
        valinit=idx0,
        valstep=1,
    )

    def update(val):
        idx = int(slider.val)
        draw_sample(idx)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()



# PLOT FOR Measurements ----------
def _random_color_from_id(label_id):
    rng = np.random.default_rng(int(label_id))
    return tuple(rng.uniform(0.05, 0.65, size=3))

def _image_extent_m(row):
    resolution = float(row["resolution"])
    width = int(row["width"])
    height = int(row["height"])
    return (0.0, width * resolution, height * resolution, 0.0)

def _pixel_to_meter_xy(x_px, y_px, resolution):
    return x_px * resolution, y_px * resolution

def _masked(img):
    return np.ma.masked_where(img == 0, img)

def _draw_landmark_mask_m(ax, row, resolution, alpha=0.35):
    x = int(row["bbox_x"])
    y = int(row["bbox_y"])
    width = int(row["bbox_width"])
    height = int(row["bbox_height"])
    mask = parse_mask_from_row(row)

    color = _random_color_from_id(int(row["label_id"]))

    rgba = np.zeros((height, width, 4), dtype=float)
    rgba[..., 0] = color[0]
    rgba[..., 1] = color[1]
    rgba[..., 2] = color[2]
    rgba[..., 3] = np.where(mask > 0, alpha, 0.0)

    x0_m = x * resolution
    x1_m = (x + width) * resolution
    y0_m = y * resolution
    y1_m = (y + height) * resolution

    ax.imshow(
        rgba,
        interpolation="nearest",
        extent=(x0_m, x1_m, y1_m, y0_m),
    )

def _draw_landmark_centroid_m(ax, row, resolution):
    cx_m, cy_m = _pixel_to_meter_xy(
        float(row["cx"]),
        float(row["cy"]),
        resolution,
    )

    ax.plot(
        cx_m,
        cy_m,
        marker="o",
        markersize=4,
        color="black",
    )

def _draw_pose_origin_m(ax, row, resolution):
    pose_x_m, pose_y_m = _pixel_to_meter_xy(
        float(row["pose_x"]),
        float(row["pose_y"]),
        resolution,
    )

    ax.plot(
        pose_x_m,
        pose_y_m,
        marker="x",
        markersize=8,
        color="red",
        markeredgewidth=2.0,
    )

    return pose_x_m, pose_y_m

def _draw_landmark_measurement_line(ax, row, pose_x_m, pose_y_m, pose_yaw, yaw_offset):
    r = float(row["z_r"])
    theta = float(row["z_theta"])

    theta_map = pose_yaw + theta + yaw_offset

    end_x_m = pose_x_m + r * np.cos(theta_map)
    end_y_m = pose_y_m + r * np.sin(theta_map)

    ax.plot(
        [pose_x_m, end_x_m],
        [pose_y_m, end_y_m],
        color="black",
        linewidth=1.2,
    )

def _draw_landmark_covariance_ellipse(ax, row, pose_x_m, pose_y_m, pose_yaw, yaw_offset, n_sigma=2.0):
    r = float(row["z_r"])
    theta = float(row["z_theta"])

    theta_map = pose_yaw + theta + yaw_offset

    R_polar = np.array([
        [float(row["R_z_rr"]),       float(row["R_z_rtheta"])],
        [float(row["R_z_thetar"]),   float(row["R_z_thetatheta"])],
    ], dtype=float)

    # Convert polar covariance to local Cartesian covariance at the landmark endpoint.
    J = np.array([
        [np.cos(theta_map), -r * np.sin(theta_map)],
        [np.sin(theta_map),  r * np.cos(theta_map)],
    ], dtype=float)

    R_xy = J @ R_polar @ J.T

    vals, vecs = np.linalg.eigh(R_xy)
    vals = np.maximum(vals, 0.0)

    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    angle_deg = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    end_x_m = pose_x_m + r * np.cos(theta_map)
    end_y_m = pose_y_m + r * np.sin(theta_map)

    ellipse = patches.Ellipse(
        (end_x_m, end_y_m),
        width=2.0 * n_sigma * np.sqrt(vals[0]),
        height=2.0 * n_sigma * np.sqrt(vals[1]),
        angle=angle_deg,
        facecolor=(0.0, 0.0, 0.0, 0.08),
        edgecolor=(0.0, 0.0, 0.0, 0.35),
        linewidth=1.0,
    )
    ax.add_patch(ellipse)

def plot_landmark_measurements(filtered_df, landmark_df, title="Landmark Measurements", yaw_offset=0.0):
    if len(filtered_df) == 0:
        raise ValueError("No filtered image samples to plot")
    if len(landmark_df) == 0:
        raise ValueError("No landmark samples to plot")

    filtered_df = filtered_df.reset_index(drop=True)

    # Group landmark rows by timestamp.
    landmark_groups = {
        float(t): group.reset_index(drop=True)
        for t, group in landmark_df.groupby("t", sort=True)
    }

    # Only keep samples that exist in both logs.
    valid_indices = []
    for i in range(len(filtered_df)):
        t = float(filtered_df.iloc[i]["t"])
        if t in landmark_groups:
            valid_indices.append(i)

    if len(valid_indices) == 0:
        raise ValueError("No matching timestamps between filtered image csv and landmark csv")

    idx0 = 0

    cmap_img = cm.get_cmap("copper").copy()
    cmap_img.set_bad(color="white")

    fig, ax = plt.subplots(1, 1, figsize=(11, 9))
    plt.subplots_adjust(bottom=0.16)

    def draw_sample(sample_idx):
        ax.clear()

        df_idx = valid_indices[sample_idx]
        filtered_row = filtered_df.iloc[df_idx]
        t = float(filtered_row["t"])
        landmark_rows = landmark_groups[t]

        filtered_image = get_processed_image_from_row(filtered_row)
        resolution = float(filtered_row["resolution"])
        extent_m = _image_extent_m(filtered_row)

        valid = filtered_image[filtered_image != 0]
        vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
        vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

        # Background filtered image in metric coordinates.
        ax.imshow(
            _masked(filtered_image),
            cmap=cmap_img,
            interpolation="nearest",
            vmin=vmin,
            vmax=vmax,
            extent=extent_m,
        )

        # Map pose / origin marker.
        pose_x_m, pose_y_m = _draw_pose_origin_m(ax, filtered_row, resolution)
        pose_yaw = float(filtered_row["pose_yaw"])

        # Overlay landmark masks, centroids, measurement rays, and covariance.
        for _, row in landmark_rows.iterrows():
            _draw_landmark_mask_m(ax, row, resolution, alpha=0.35)
            _draw_landmark_centroid_m(ax, row, resolution)
            _draw_landmark_measurement_line(ax, row, pose_x_m, pose_y_m, pose_yaw, yaw_offset)
            _draw_landmark_covariance_ellipse(ax, row, pose_x_m, pose_y_m, pose_yaw, yaw_offset, n_sigma=2.0)

        ax.set_title("Landmark Measurements")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")

        fig.suptitle(f"{title}")

    draw_sample(idx0)

    slider_ax = fig.add_axes([0.15, 0.06, 0.7, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Sample",
        valmin=0,
        valmax=len(valid_indices) - 1,
        valinit=idx0,
        valstep=1,
    )

    def update(val):
        idx = int(slider.val)
        draw_sample(idx)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()



# PLOT FOR Estimated Height ----------
def _draw_landmark_bbox_m(ax, row, resolution, linewidth=1.6):
    x = int(row["bbox_x"])
    y = int(row["bbox_y"])
    width = int(row["bbox_width"])
    height = int(row["bbox_height"])

    color = _random_color_from_id(int(row["label_id"]))

    rect = patches.Rectangle(
        (x * resolution, y * resolution),
        width * resolution,
        height * resolution,
        linewidth=linewidth,
        edgecolor=color,
        facecolor=(color[0], color[1], color[2], 0.08),
    )
    ax.add_patch(rect)

def _draw_landmark_id_and_height_m(ax, row, resolution):
    label_id = int(row["label_id"])
    x_m = float(row["bbox_x"]) * resolution
    y_m = float(row["bbox_y"]) * resolution

    height_txt = "nan"
    if "height" in row and pd.notna(row["height"]):
        height_txt = f"{float(row['height']):.2f} m"

    color = _random_color_from_id(label_id)

    ax.text(
        x_m,
        max(y_m - 0.35 * resolution, 0.0),
        f"ID {label_id}, h={height_txt}",
        color=color,
        fontsize=8,
        ha="left",
        va="bottom",
        bbox=dict(
            facecolor=(1, 1, 1, 0.70),
            edgecolor="none",
            pad=1.5,
        ),
    )

def plot_landmark_height(filtered_df, landmark_df, title="Estimated Landmark Height"):
    if len(filtered_df) == 0:
        raise ValueError("No filtered image samples to plot")
    if len(landmark_df) == 0:
        raise ValueError("No landmark samples to plot")

    filtered_df = filtered_df.reset_index(drop=True)

    landmark_groups = {
        float(t): group.reset_index(drop=True)
        for t, group in landmark_df.groupby("t", sort=True)
    }

    valid_indices = []
    for i in range(len(filtered_df)):
        t = float(filtered_df.iloc[i]["t"])
        if t in landmark_groups:
            valid_indices.append(i)

    if len(valid_indices) == 0:
        raise ValueError("No matching timestamps between filtered image csv and landmark csv")

    idx0 = 0

    cmap_img = cm.get_cmap("copper").copy()
    cmap_img.set_bad(color="white")

    fig, ax = plt.subplots(1, 1, figsize=(12, 9))
    plt.subplots_adjust(bottom=0.16)

    def draw_sample(sample_idx):
        ax.clear()

        df_idx = valid_indices[sample_idx]
        filtered_row = filtered_df.iloc[df_idx]
        t = float(filtered_row["t"])
        landmark_rows = landmark_groups[t]

        filtered_image = get_processed_image_from_row(filtered_row)
        resolution = float(filtered_row["resolution"])
        extent_m = _image_extent_m(filtered_row)

        valid = filtered_image[filtered_image != 0]
        vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
        vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

        ax.imshow(
            _masked(filtered_image),
            cmap=cmap_img,
            interpolation="nearest",
            vmin=vmin,
            vmax=vmax,
            extent=extent_m,
        )

        for _, row in landmark_rows.iterrows():
            _draw_landmark_mask_m(ax, row, resolution, alpha=0.35)
            _draw_landmark_bbox_m(ax, row, resolution)
            _draw_landmark_centroid_m(ax, row, resolution)
            _draw_landmark_id_and_height_m(ax, row, resolution)

        ax.set_title("Estimated Landmark Height")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")

        fig.suptitle(title)

    draw_sample(idx0)

    slider_ax = fig.add_axes([0.15, 0.06, 0.7, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Sample",
        valmin=0,
        valmax=len(valid_indices) - 1,
        valinit=idx0,
        valstep=1,
    )

    def update(val):
        idx = int(slider.val)
        draw_sample(idx)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()



# PLOT FOR Descriptors ----------
def _get_descriptor_columns(df):
    cols = [
        "mean_intensity",
        "std",
        "contrast",
        "entropy",
        "area",
        "weak_polar_r",
        "weak_polar_theta",
        "height",
        "radial_intensity_gradient",
    ]
    return [c for c in cols if c in df.columns]

def _get_descriptor_matrix(df, descriptor_cols):
    X = df[descriptor_cols].copy().astype(float)

    # Drop columns that are fully constant or invalid for this sample.
    keep_cols = []
    for c in descriptor_cols:
        values = X[c].to_numpy(dtype=float)
        if np.all(np.isfinite(values)) and np.nanstd(values) > 0.0:
            keep_cols.append(c)

    if len(keep_cols) == 0:
        return np.zeros((len(df), 2)), []

    X = X[keep_cols].to_numpy(dtype=float)

    # Standardize so one large-scale descriptor does not dominate.
    mu = np.mean(X, axis=0)
    sigma = np.std(X, axis=0)
    sigma[sigma == 0.0] = 1.0
    Xn = (X - mu) / sigma

    return Xn, keep_cols

def _project_descriptors_pca_2d(X):
    if X.shape[0] == 0:
        return np.zeros((0, 2)), np.array([0.0, 0.0])

    if X.shape[0] == 1:
        return np.column_stack([np.zeros(1), np.zeros(1)]), np.array([1.0, 0.0])

    if X.shape[1] == 1:
        return np.column_stack([X[:, 0], np.zeros(X.shape[0])]), np.array([1.0, 0.0])

    Xc = X - np.mean(X, axis=0, keepdims=True)
    U, S, Vt = np.linalg.svd(Xc, full_matrices=False)
    coords = Xc @ Vt[:2].T

    var = (S ** 2) / max(X.shape[0] - 1, 1)
    if len(var) == 1:
        explained = np.array([1.0, 0.0])
    else:
        total = np.sum(var)
        explained = np.array([
            var[0] / total if total > 0 else 0.0,
            var[1] / total if len(var) > 1 and total > 0 else 0.0,
        ])

    return coords, explained

def _compute_descriptor_uniqueness(coords):
    n = coords.shape[0]
    if n == 0:
        return np.array([])

    if n == 1:
        return np.array([0.0])

    dists = np.sqrt(np.sum((coords[:, None, :] - coords[None, :, :]) ** 2, axis=2))
    np.fill_diagonal(dists, np.inf)

    # Nearest-neighbor distance = simple uniqueness score.
    score = np.min(dists, axis=1)

    max_score = np.max(score)
    if max_score > 0:
        score = score / max_score

    return score

def _draw_landmark_id_m(ax, row, resolution):
    label_id = int(row["label_id"])
    cx_m, cy_m = _pixel_to_meter_xy(
        float(row["cx"]),
        float(row["cy"]),
        resolution,
    )

    ax.text(
        cx_m + 0.25 * resolution,
        cy_m - 0.25 * resolution,
        f"ID {label_id}",
        color="black",
        fontsize=8,
        ha="left",
        va="bottom",
        bbox=dict(
            facecolor=(1, 1, 1, 0.65),
            edgecolor="none",
            pad=1.5,
        ),
    )

def _draw_descriptor_space(ax, landmark_rows):
    descriptor_cols = _get_descriptor_columns(landmark_rows)
    X, used_cols = _get_descriptor_matrix(landmark_rows, descriptor_cols)

    if X.shape[0] == 0 or len(used_cols) == 0:
        ax.text(0.5, 0.5, "No valid descriptor data", ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Descriptor Space")
        ax.set_xlabel("PC1")
        ax.set_ylabel("PC2")
        return

    coords, explained = _project_descriptors_pca_2d(X)
    uniqueness = _compute_descriptor_uniqueness(coords)

    for i, (_, row) in enumerate(landmark_rows.iterrows()):
        label_id = int(row["label_id"])
        color = _random_color_from_id(label_id)

        marker_size = 70 + 180 * uniqueness[i]
        text_offset = (marker_size**0.5)/2 + 3

        ax.scatter(
            coords[i, 0],
            coords[i, 1],
            s=marker_size,
            color=color,
            edgecolors="black",
            linewidths=0.8,
            alpha=0.85,
        )

        ax.annotate(
            f"{label_id}",
            xy=(coords[i, 0], coords[i, 1]),
            xytext=(text_offset, 0),
            textcoords="offset points",
            fontsize=9,
            color="black",
            ha="left",
            va="center",
        )

    ax.set_title("Principal Component Analysis of Landmark Descriptor Space")
    ax.set_xlabel(f"PC1 ({100.0 * explained[0]:.1f}% var)")
    ax.set_ylabel(f"PC2 ({100.0 * explained[1]:.1f}% var)")
    ax.grid(True, alpha=0.25)

    if coords.shape[0] > 1:
        xpad = 0.15 * max(np.ptp(coords[:, 0]), 1.0)
        ypad = 0.15 * max(np.ptp(coords[:, 1]), 1.0)
        ax.set_xlim(np.min(coords[:, 0]) - xpad, np.max(coords[:, 0]) + xpad)
        ax.set_ylim(np.min(coords[:, 1]) - ypad, np.max(coords[:, 1]) + ypad)

    # Small helper text so you know what this plot means.
    used_txt = ", ".join(used_cols)
    ax.text(
        0.02,
        0.98,
        "Distance in this space = descriptor difference\n"
        "Bigger marker = more unique vs nearest landmark\n"
        f"Used: {used_txt}",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=8,
        bbox=dict(facecolor=(1, 1, 1, 0.75), edgecolor="none", pad=2.0),
    )

def plot_landmark_descriptors(filtered_df, landmark_df, title="Landmark Descriptors"):
    if len(filtered_df) == 0:
        raise ValueError("No filtered image samples to plot")
    if len(landmark_df) == 0:
        raise ValueError("No landmark samples to plot")

    filtered_df = filtered_df.reset_index(drop=True)

    landmark_groups = {
        float(t): group.reset_index(drop=True)
        for t, group in landmark_df.groupby("t", sort=True)
    }

    valid_indices = []
    for i in range(len(filtered_df)):
        t = float(filtered_df.iloc[i]["t"])
        if t in landmark_groups:
            valid_indices.append(i)

    if len(valid_indices) == 0:
        raise ValueError("No matching timestamps between filtered image csv and landmark csv")

    idx0 = 0

    cmap_img = cm.get_cmap("copper").copy()
    cmap_img.set_bad(color="white")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    plt.subplots_adjust(bottom=0.16, wspace=0.22)

    ax_left, ax_right = axes

    def draw_sample(sample_idx):
        ax_left.clear()
        ax_right.clear()

        df_idx = valid_indices[sample_idx]
        filtered_row = filtered_df.iloc[df_idx]
        t = float(filtered_row["t"])
        landmark_rows = landmark_groups[t]

        filtered_image = get_processed_image_from_row(filtered_row)
        resolution = float(filtered_row["resolution"])
        extent_m = _image_extent_m(filtered_row)

        valid = filtered_image[filtered_image != 0]
        vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
        vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

        # Left: filtered image + landmark masks + ID text.
        ax_left.imshow(
            _masked(filtered_image),
            cmap=cmap_img,
            interpolation="nearest",
            vmin=vmin,
            vmax=vmax,
            extent=extent_m,
        )

        for _, row in landmark_rows.iterrows():
            _draw_landmark_mask_m(ax_left, row, resolution, alpha=0.35)
            _draw_landmark_centroid_m(ax_left, row, resolution)
            _draw_landmark_id_m(ax_left, row, resolution)

        ax_left.set_title("Landmarks on Filtered Map")
        ax_left.set_xlabel("x [m]")
        ax_left.set_ylabel("y [m]")
        ax_left.set_aspect("equal")

        # Right: descriptor-space projection.
        _draw_descriptor_space(ax_right, landmark_rows)

        fig.suptitle(f"{title}")

    draw_sample(idx0)

    slider_ax = fig.add_axes([0.15, 0.06, 0.7, 0.04])
    slider = Slider(
        ax=slider_ax,
        label="Sample",
        valmin=0,
        valmax=len(valid_indices) - 1,
        valinit=idx0,
        valstep=1,
    )

    def update(val):
        idx = int(slider.val)
        draw_sample(idx)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()
