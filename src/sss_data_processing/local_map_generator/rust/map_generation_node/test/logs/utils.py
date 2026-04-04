# utils.py
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.widgets import Slider
import numpy as np



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
        df = df.iloc[start_sample:end_sample].reset_index(drop=True)

    if start_t is not None:
        df = df[df["t"] >= start_t]

    if end_t is not None:
        df = df[df["t"] <= end_t]

    df = df.reset_index(drop=True)
    return df



# Q_m HELPERS ----------
def parse_qm_string(cell_map_m_str):
    if pd.isna(cell_map_m_str) or not str(cell_map_m_str).strip():
        return np.array([]), np.array([])

    xs = []
    ys = []

    for item in str(cell_map_m_str).split("|"):
        item = item.strip()
        if not item:
            continue

        parts = item.split()
        if len(parts) != 4:
            continue

        xs.append(float(parts[0]))
        ys.append(float(parts[1]))

    return np.array(xs), np.array(ys)


def get_closest_sample_idx(df, t_target):
    return (df["t"] - t_target).abs().idxmin()


def get_closest_pose_row(pose_df, t_target):
    idx = (pose_df["t"] - t_target).abs().idxmin()
    return pose_df.loc[idx]


def get_rows_up_to_time(df, t_target):
    return df[df["t"] <= t_target].reset_index(drop=True)



# SWATH HELPERS ----------
def parse_array(col, target_len=None):
    rows = [list(map(float, s.split())) for s in col]

    if target_len is None:
        target_len = max(len(r) for r in rows)

    out = np.zeros((len(rows), target_len))
    for i, r in enumerate(rows):
        out[i, :len(r)] = r[:target_len]

    return out



# COMBINED PLOT ----------
def plot_qm_and_swath_interactive(
    cell_map_m_df,
    pose_df,
    swath_df,
    title="Q_m and Processed Swath",
    q_ylabel="y [m]",
    q_xlabel="x [m]",
    swath_xlabel="Across track",
    zero_color="white",
    cmap_name="copper",
):
    pose_path_color = "gray"
    q_m_color = "blue"
    pose_color = "orange"
    arrow_color = "black"
    time_line_color = "red"
    
    yaw_offset = -np.pi/2.0

    port = parse_array(swath_df["port"])
    stb = parse_array(swath_df["starboard"])
    swath = np.hstack([port, stb])

    swath_masked = np.ma.masked_where(swath == 0, swath)

    cmap = cm.get_cmap(cmap_name).copy()
    cmap.set_bad(color=zero_color)

    valid = swath[swath != 0]
    vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
    vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

    # Use one common time origin for everything.
    # This keeps the relative-time axis consistent even if one file starts later.
    t0 = min(
        float(cell_map_m_df["t"].iloc[0]),
        float(pose_df["t"].iloc[0]),
        float(swath_df["t"].iloc[0]),
    )

    swath_t_min = float(swath_df["t"].iloc[0]) - t0
    swath_t_max = float(swath_df["t"].iloc[-1]) - t0

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    plt.subplots_adjust(bottom=0.18)

    # right side fixed swath
    axes[1].imshow(
        swath_masked,
        aspect="auto",
        cmap=cmap,
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
        extent=[0, swath.shape[1], swath_t_max, swath_t_min],
    )
    axes[1].set_title("Processed Swath")
    axes[1].set_ylabel("t [s]")
    axes[1].set_xlabel(swath_xlabel)

    # left side initial sample
    sample_idx = len(cell_map_m_df) - 1

    cell_map_m_row = cell_map_m_df.iloc[sample_idx]
    t_abs = float(cell_map_m_row["t"])
    t_rel = t_abs - t0
    qx, qy = parse_qm_string(cell_map_m_row["cell_map_m"])
    pose_row = get_closest_pose_row(pose_df, t_abs)

    pose_x = float(pose_row["x"])
    pose_y = float(pose_row["y"])
    yaw = float(pose_row["yaw"]) + yaw_offset

    # background: all pose positions
    axes[0].scatter(
        pose_df["x"].to_numpy(),
        pose_df["y"].to_numpy(),
        s=8,
        color=pose_path_color,
        alpha=0.25,
        label="Pose path",
    )

    scatter = axes[0].scatter(
        qx, qy,
        s=6,
        color=q_m_color,
        label=f"Q_m (n={len(qx)})",
    )

    pose_scatter = axes[0].scatter(
        [pose_x], [pose_y],
        s=80,
        marker="x",
        color=pose_color,
        label="Pose",
    )

    arrow_len = 2.0
    dx = arrow_len * np.cos(yaw)
    dy = arrow_len * np.sin(yaw)
    arrow = axes[0].arrow(
        pose_x, pose_y, dx, dy,
        head_width=0.35,
        head_length=0.55,
        length_includes_head=True,
        color=arrow_color,
    )

    # red horizontal time marker on swath image
    time_line = axes[1].axhline(
        y=t_rel,
        linewidth=1.5,
        color=time_line_color,
        label="Selected time",
    )

    axes[0].set_title(f"Q_m at t={t_rel:.6f}")
    axes[0].set_ylabel(q_ylabel)
    axes[0].set_xlabel(q_xlabel)
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].legend()

    # slider
    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.04])
    slider = Slider(
        ax=ax_slider,
        label="t [s]",
        valmin=float(cell_map_m_df["t"].iloc[0]) - t0,
        valmax=float(cell_map_m_df["t"].iloc[-1]) - t0,
        valinit=t_rel,
    )

    def update(val):
        nonlocal arrow

        t_rel = float(slider.val)
        t_abs = t_rel + t0

        idx = get_closest_sample_idx(cell_map_m_df, t_abs)
        cell_map_m_row = cell_map_m_df.loc[idx]

        qx, qy = parse_qm_string(cell_map_m_row["cell_map_m"])
        pose_row = get_closest_pose_row(pose_df, t_abs)

        pose_x = float(pose_row["x"])
        pose_y = float(pose_row["y"])
        yaw = float(pose_row["yaw"]) + yaw_offset

        if len(qx) > 0:
            scatter.set_offsets(np.column_stack([qx, qy]))
        else:
            scatter.set_offsets(np.empty((0, 2)))

        pose_scatter.set_offsets(np.array([[pose_x, pose_y]]))

        arrow.remove()
        dx = arrow_len * np.cos(yaw)
        dy = arrow_len * np.sin(yaw)
        arrow = axes[0].arrow(
            pose_x, pose_y, dx, dy,
            head_width=0.35,
            head_length=0.55,
            length_includes_head=True,
            color=arrow_color,
        )

        time_line.set_ydata([t_rel, t_rel])

        scatter.set_label(f"Q_m (n={len(qx)})")
        axes[0].set_title(f"Q_m at t={t_rel:.6f}")
        axes[0].legend()
        fig.canvas.draw_idle()

    slider.on_changed(update)

    fig.suptitle(title)
    plt.show()
