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
        df = df.iloc[start_sample:end_sample].reset_index(drop=True)

    if start_t is not None:
        df = df[df["t"] >= start_t]

    if end_t is not None:
        df = df[df["t"] <= end_t]

    df = df.reset_index(drop=True)
    return df



# CELL MAP M HELPERS ----------
def parse_cell_map_m_string(cell_map_m_str):
    if pd.isna(cell_map_m_str) or not str(cell_map_m_str).strip():
        return np.array([]), np.array([]), np.array([]), np.array([])

    xs = []
    ys = []
    ps = []
    vs = []

    for item in str(cell_map_m_str).split("|"):
        item = item.strip()
        if not item:
            continue

        parts = item.split()

        # Expected:
        # q_m.x q_m.y p_m v_m
        if len(parts) != 4:
            continue

        xs.append(float(parts[0]))
        ys.append(float(parts[1]))
        ps.append(float(parts[2]))
        vs.append(float(parts[3]))

    return (
        np.array(xs),
        np.array(ys),
        np.array(ps),
        np.array(vs),
    )


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



# COMMON PLOT ----------
def plot_cell_map_m_and_swath_interactive(
    cell_map_m_df,
    pose_df,
    swath_df,
    left_mode="q_m",   # "q_m" or "p_m"
    title="CellMapM and Processed Swath",
    left_ylabel="y [m]",
    left_xlabel="x [m]",
    swath_xlabel="Across track",
    zero_color="white",
    cmap_name="copper",
    p_m_cmap_name="viridis",
    v_m_cmap_name="copper",
):
    pose_path_color = "gray"
    q_m_color = "blue"
    pose_color = "orange"
    arrow_color = "black"
    time_line_color = "red"

    yaw_offset = -np.pi / 2.0

    # Build swath image
    port = parse_array(swath_df["port"])
    stb = parse_array(swath_df["starboard"])
    swath = np.hstack([port, stb])

    swath_masked = np.ma.masked_where(swath == 0, swath)

    swath_cmap = cm.get_cmap(cmap_name).copy()
    swath_cmap.set_bad(color=zero_color)

    valid = swath[swath != 0]
    vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
    vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

    # Common relative time origin
    t0 = min(
        float(cell_map_m_df["t"].iloc[0]),
        float(pose_df["t"].iloc[0]),
        float(swath_df["t"].iloc[0]),
    )

    swath_t_min = float(swath_df["t"].iloc[0]) - t0
    swath_t_max = float(swath_df["t"].iloc[-1]) - t0

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    plt.subplots_adjust(bottom=0.18)

    # Right side: fixed swath
    axes[1].imshow(
        swath_masked,
        aspect="auto",
        cmap=swath_cmap,
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
        extent=[0, swath.shape[1], swath_t_max, swath_t_min],
    )
    axes[1].set_title("Processed Swath")
    axes[1].set_ylabel("t [s]")
    axes[1].set_xlabel(swath_xlabel)

    # Initial sample
    sample_idx = len(cell_map_m_df) - 1

    cell_map_m_row = cell_map_m_df.iloc[sample_idx]
    t_abs = float(cell_map_m_row["t"])
    t_rel = t_abs - t0

    qx, qy, pm, vm = parse_cell_map_m_string(cell_map_m_row["cell_map_m"])
    pose_row = get_closest_pose_row(pose_df, t_abs)

    pose_x = float(pose_row["x"])
    pose_y = float(pose_row["y"])
    yaw = float(pose_row["yaw"]) + yaw_offset

    # Background pose path
    axes[0].scatter(
        pose_df["x"].to_numpy(),
        pose_df["y"].to_numpy(),
        s=8,
        color=pose_path_color,
        alpha=0.25,
        label="Pose path",
    )

    colorbar = None

    if left_mode == "q_m":
        scatter = axes[0].scatter(
            qx, qy,
            s=6,
            color=q_m_color,
            label=f"Q_m (n={len(qx)})",
        )
        left_title = f"Q_m at t={t_rel:.6f}"
    elif left_mode == "p_m":
        scatter = axes[0].scatter(
            qx, qy,
            s=10,
            c=pm if len(pm) > 0 else np.array([]),
            cmap=p_m_cmap_name,
            label=f"P_m (n={len(qx)})",
        )
        colorbar = fig.colorbar(scatter, ax=axes[0])
        colorbar.set_label("P_m")
        left_title = f"P_m at t={t_rel:.6f}"
    elif left_mode == "v_m":
        scatter = axes[0].scatter(
            qx, qy,
            s=10,
            c=vm if len(vm) > 0 else np.array([]),
            cmap=v_m_cmap_name,
            label=f"V_m (n={len(qx)})",
        )
        colorbar = fig.colorbar(scatter, ax=axes[0])
        colorbar.set_label("V_m")
        left_title = f"V_m at t={t_rel:.6f}"
    else:
        raise ValueError("left_mode must be 'q_m', 'p_m', or 'v_m'")

    pose_scatter = axes[0].scatter(
        [pose_x],
        [pose_y],
        s=80,
        marker="x",
        color=pose_color,
        label="Pose",
    )

    arrow_len = 2.0
    dx = arrow_len * np.cos(yaw)
    dy = arrow_len * np.sin(yaw)
    arrow = axes[0].arrow(
        pose_x,
        pose_y,
        dx,
        dy,
        head_width=0.35,
        head_length=0.55,
        length_includes_head=True,
        color=arrow_color,
    )

    time_line = axes[1].axhline(
        y=t_rel,
        linewidth=1.5,
        color=time_line_color,
        label="Selected time",
    )

    axes[0].set_title(left_title)
    axes[0].set_ylabel(left_ylabel)
    axes[0].set_xlabel(left_xlabel)
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].legend()

    # Slider by time
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

        qx, qy, pm, vm = parse_cell_map_m_string(cell_map_m_row["cell_map_m"])
        pose_row = get_closest_pose_row(pose_df, t_abs)

        pose_x = float(pose_row["x"])
        pose_y = float(pose_row["y"])
        yaw = float(pose_row["yaw"]) + yaw_offset

        if len(qx) > 0:
            scatter.set_offsets(np.column_stack([qx, qy]))
        else:
            scatter.set_offsets(np.empty((0, 2)))

        if left_mode == "p_m":
            if len(pm) > 0:
                scatter.set_array(pm)
                scatter.set_clim(vmin=float(pm.min()), vmax=float(pm.max()))
            else:
                scatter.set_array(np.array([]))
        elif left_mode == "v_m":
            if len(vm) > 0:
                scatter.set_array(vm)
                scatter.set_clim(vmin=float(vm.min()), vmax=float(vm.max()))
            else:
                scatter.set_array(np.array([]))

        pose_scatter.set_offsets(np.array([[pose_x, pose_y]]))

        arrow.remove()
        dx = arrow_len * np.cos(yaw)
        dy = arrow_len * np.sin(yaw)
        arrow = axes[0].arrow(
            pose_x,
            pose_y,
            dx,
            dy,
            head_width=0.35,
            head_length=0.55,
            length_includes_head=True,
            color=arrow_color,
        )

        time_line.set_ydata([t_rel, t_rel])

        if left_mode == "q_m":
            scatter.set_label(f"Q_m (n={len(qx)})")
            axes[0].set_title(f"Q_m at t={t_rel:.6f}")
        elif left_mode == "p_m":
            scatter.set_label(f"P_m (n={len(qx)})")
            axes[0].set_title(f"P_m at t={t_rel:.6f}")
        else:
            scatter.set_label(f"V_m (n={len(qx)})")
            axes[0].set_title(f"V_m at t={t_rel:.6f}")

        axes[0].legend()
        fig.canvas.draw_idle()

    slider.on_changed(update)

    fig.suptitle(title)
    plt.show()


# WRAPPERS ----------
def plot_qm_and_swath_interactive(
    cell_map_m_df,
    pose_df,
    swath_df,
    title="Q_m and Processed Swath",
):
    plot_cell_map_m_and_swath_interactive(
        cell_map_m_df=cell_map_m_df,
        pose_df=pose_df,
        swath_df=swath_df,
        left_mode="q_m",
        title=title,
    )

def plot_pm_and_swath_interactive(
    cell_map_m_df,
    pose_df,
    swath_df,
    title="P_m and Processed Swath",
):
    plot_cell_map_m_and_swath_interactive(
        cell_map_m_df=cell_map_m_df,
        pose_df=pose_df,
        swath_df=swath_df,
        left_mode="p_m",
        title=title,
    )

def plot_vm_and_swath_interactive(
    cell_map_m_df,
    pose_df,
    swath_df,
    title="V_m and Processed Swath",
):
    plot_cell_map_m_and_swath_interactive(
        cell_map_m_df=cell_map_m_df,
        pose_df=pose_df,
        swath_df=swath_df,
        left_mode="v_m",
        title=title,
    )

# CHUNK MAP HELPERS ----------
def parse_chunk_map_string(chunk_map_str, chunk_size):
    if pd.isna(chunk_map_str) or not str(chunk_map_str).strip():
        return {
            "x_world": np.array([]),
            "y_world": np.array([]),
            "p": np.array([]),
            "v": np.array([]),
            "chunk_x": np.array([]),
            "chunk_y": np.array([]),
            "chunk_age": np.array([]),
            "chunk_boxes": [],
        }

    x_world = []
    y_world = []
    p_vals = []
    v_vals = []

    chunk_xs = []
    chunk_ys = []
    chunk_ages = []

    chunk_info = {}

    for item in str(chunk_map_str).split("|"):
        item = item.strip()
        if not item:
            continue

        parts = item.split()

        # Expected:
        # chunk_x chunk_y chunk_age cell_x cell_y p v
        if len(parts) != 7:
            continue

        chunk_x = int(parts[0])
        chunk_y = int(parts[1])
        chunk_age = int(parts[2])
        cell_x = int(parts[3])
        cell_y = int(parts[4])
        p_val = float(parts[5])
        v_val = float(parts[6])

        world_x = chunk_x * chunk_size + cell_x
        world_y = chunk_y * chunk_size + cell_y

        x_world.append(world_x)
        y_world.append(world_y)
        p_vals.append(p_val)
        v_vals.append(v_val)

        chunk_xs.append(chunk_x)
        chunk_ys.append(chunk_y)
        chunk_ages.append(chunk_age)

        chunk_info[(chunk_x, chunk_y)] = chunk_age

    chunk_boxes = []
    for (chunk_x, chunk_y), chunk_age in chunk_info.items():
        chunk_boxes.append({
            "x": chunk_x * chunk_size,
            "y": chunk_y * chunk_size,
            "age": chunk_age,
        })

    return {
        "x_world": np.array(x_world, dtype=float),
        "y_world": np.array(y_world, dtype=float),
        "p": np.array(p_vals, dtype=float),
        "v": np.array(v_vals, dtype=float),
        "chunk_x": np.array(chunk_xs, dtype=int),
        "chunk_y": np.array(chunk_ys, dtype=int),
        "chunk_age": np.array(chunk_ages, dtype=int),
        "chunk_boxes": chunk_boxes,
    }


def _add_chunk_overlays(ax, chunk_boxes, chunk_size):
    if not chunk_boxes:
        return []

    max_age = max(box["age"] for box in chunk_boxes)
    max_age = max(max_age, 1)

    patch_list = []

    for box in chunk_boxes:
        age_norm = box["age"] / max_age
        gray = 0.85 - 0.55 * age_norm
        gray = np.clip(gray, 0.2, 0.9)

        rect = patches.Rectangle(
            (box["x"], box["y"]),
            chunk_size,
            chunk_size,
            linewidth=1.2,
            edgecolor=(gray, gray, gray, 0.9),
            facecolor=(gray, gray, gray, 0.08 + 0.12 * age_norm),
        )
        ax.add_patch(rect)
        patch_list.append(rect)

    return patch_list


def _remove_chunk_overlays(patch_list):
    for patch in patch_list:
        patch.remove()



# CHUNK MAP PLOT ----------
def plot_chunk_map_pv_interactive(
    chunk_map_df,
    pose_df,
    chunk_size,
    map_resolution,
    title="Chunk Map: V and P",
    ylabel="y [cells]",
    xlabel="x [cells]",
    p_cmap_name="viridis",
    v_cmap_name="copper",
):
    pose_path_color = "gray"
    pose_color = "orange"
    arrow_color = "black"

    yaw_offset = -np.pi / 2.0

    t0 = min(
        float(chunk_map_df["t"].iloc[0]),
        float(pose_df["t"].iloc[0]),
    )

    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    plt.subplots_adjust(bottom=0.18)

    sample_idx = len(chunk_map_df) - 1

    row = chunk_map_df.iloc[sample_idx]
    t_abs = float(row["t"])
    t_rel = t_abs - t0

    parsed = parse_chunk_map_string(row["chunk_map"], chunk_size)
    pose_row = get_closest_pose_row(pose_df, t_abs)

    pose_path_x = pose_df["x"].to_numpy()/map_resolution
    pose_path_y = pose_df["y"].to_numpy()/map_resolution

    pose_x = float(pose_row["x"])/map_resolution
    pose_y = float(pose_row["y"])/map_resolution
    yaw = float(pose_row["yaw"]) + yaw_offset

    # pose path background on both
    for ax in axes:
        ax.scatter(
            pose_path_x,
            pose_path_y,
            s=8,
            color=pose_path_color,
            alpha=0.25,
            label="Pose path",
        )

    # Left: V
    scatter_v = axes[0].scatter(
        parsed["x_world"],
        parsed["y_world"],
        s=18,
        c=parsed["v"] if len(parsed["v"]) > 0 else np.array([]),
        cmap=v_cmap_name,
        marker="s",
        label=f"V (n={len(parsed['v'])})",
    )
    colorbar_v = fig.colorbar(scatter_v, ax=axes[0])
    colorbar_v.set_label("V")

    # Right: P
    scatter_p = axes[1].scatter(
        parsed["x_world"],
        parsed["y_world"],
        s=18,
        c=parsed["p"] if len(parsed["p"]) > 0 else np.array([]),
        cmap=p_cmap_name,
        marker="s",
        label=f"P (n={len(parsed['p'])})",
    )
    colorbar_p = fig.colorbar(scatter_p, ax=axes[1])
    colorbar_p.set_label("P")

    # chunk overlays
    chunk_patches_v = _add_chunk_overlays(axes[0], parsed["chunk_boxes"], chunk_size)
    chunk_patches_p = _add_chunk_overlays(axes[1], parsed["chunk_boxes"], chunk_size)

    # pose + heading on both
    pose_scatter_v = axes[0].scatter(
        [pose_x], [pose_y],
        s=80,
        marker="x",
        color=pose_color,
        label="Pose",
    )
    pose_scatter_p = axes[1].scatter(
        [pose_x], [pose_y],
        s=80,
        marker="x",
        color=pose_color,
        label="Pose",
    )

    arrow_len = 2.0
    dx = arrow_len * np.cos(yaw)
    dy = arrow_len * np.sin(yaw)

    arrow_v = axes[0].arrow(
        pose_x, pose_y, dx, dy,
        head_width=0.35,
        head_length=0.55,
        length_includes_head=True,
        color=arrow_color,
    )
    arrow_p = axes[1].arrow(
        pose_x, pose_y, dx, dy,
        head_width=0.35,
        head_length=0.55,
        length_includes_head=True,
        color=arrow_color,
    )

    axes[0].set_title(f"V at t={t_rel:.6f}")
    axes[1].set_title(f"P at t={t_rel:.6f}")

    for ax in axes:
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)
        ax.axis("equal")
        ax.grid(False)
        ax.legend()

    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.04])
    slider = Slider(
        ax=ax_slider,
        label="t [s]",
        valmin=float(chunk_map_df["t"].iloc[0]) - t0,
        valmax=float(chunk_map_df["t"].iloc[-1]) - t0,
        valinit=t_rel,
    )

    def update(val):
        nonlocal arrow_v, arrow_p, chunk_patches_v, chunk_patches_p

        t_rel = float(slider.val)
        t_abs = t_rel + t0

        idx = get_closest_sample_idx(chunk_map_df, t_abs)
        row = chunk_map_df.loc[idx]

        parsed = parse_chunk_map_string(row["chunk_map"], chunk_size)
        pose_row = get_closest_pose_row(pose_df, t_abs)

        pose_x = float(pose_row["x"])/map_resolution
        pose_y = float(pose_row["y"])/map_resolution
        yaw = float(pose_row["yaw"]) + yaw_offset

        if len(parsed["x_world"]) > 0:
            offsets = np.column_stack([parsed["x_world"], parsed["y_world"]])
        else:
            offsets = np.empty((0, 2))

        scatter_v.set_offsets(offsets)
        scatter_p.set_offsets(offsets)

        if len(parsed["v"]) > 0:
            scatter_v.set_array(parsed["v"])
            scatter_v.set_clim(vmin=float(parsed["v"].min()), vmax=float(parsed["v"].max()))
        else:
            scatter_v.set_array(np.array([]))

        if len(parsed["p"]) > 0:
            scatter_p.set_array(parsed["p"])
            scatter_p.set_clim(vmin=float(parsed["p"].min()), vmax=float(parsed["p"].max()))
        else:
            scatter_p.set_array(np.array([]))

        pose_scatter_v.set_offsets(np.array([[pose_x, pose_y]]))
        pose_scatter_p.set_offsets(np.array([[pose_x, pose_y]]))

        arrow_v.remove()
        arrow_p.remove()

        dx = arrow_len * np.cos(yaw)
        dy = arrow_len * np.sin(yaw)

        arrow_v = axes[0].arrow(
            pose_x, pose_y, dx, dy,
            head_width=0.35,
            head_length=0.55,
            length_includes_head=True,
            color=arrow_color,
        )
        arrow_p = axes[1].arrow(
            pose_x, pose_y, dx, dy,
            head_width=0.35,
            head_length=0.55,
            length_includes_head=True,
            color=arrow_color,
        )

        _remove_chunk_overlays(chunk_patches_v)
        _remove_chunk_overlays(chunk_patches_p)

        chunk_patches_v = _add_chunk_overlays(axes[0], parsed["chunk_boxes"], chunk_size)
        chunk_patches_p = _add_chunk_overlays(axes[1], parsed["chunk_boxes"], chunk_size)

        scatter_v.set_label(f"V (n={len(parsed['v'])})")
        scatter_p.set_label(f"P (n={len(parsed['p'])})")

        axes[0].set_title(f"V at t={t_rel:.6f}")
        axes[1].set_title(f"P at t={t_rel:.6f}")

        axes[0].legend()
        axes[1].legend()

        fig.canvas.draw_idle()

    slider.on_changed(update)

    fig.suptitle(title)
    plt.show()

# MAP HELPERS ----------
def parse_map_string(map_str, width, height):
    if pd.isna(map_str) or not str(map_str).strip() or width <= 0 or height <= 0:
        return np.zeros((0, 0), dtype=np.uint8)

    values = np.fromstring(str(map_str), sep=" ", dtype=np.uint8)

    expected = width * height
    if values.size < expected:
        padded = np.zeros(expected, dtype=np.uint8)
        padded[:values.size] = values
        values = padded
    elif values.size > expected:
        values = values[:expected]

    return values.reshape((height, width))

# MAP PLOT ----------
def plot_map_interactive(
    map_df,
    map_resolution,
    title="Map",
    xlabel="x [cells]",
    ylabel="y [cells]",
    cmap_name="copper",
    zero_color="white",
):
    pose_color = "orange"
    arrow_color = "black"
    yaw_offset = -np.pi / 2.0

    t0 = float(map_df["t"].iloc[0])

    sample_idx = len(map_df) - 1
    row = map_df.iloc[sample_idx]

    t_abs = float(row["t"])
    t_rel = t_abs - t0

    width = int(row["width"])
    height = int(row["height"])
    M = parse_map_string(row["map"], width, height)

    pose_x = float(row["pose_x"])
    pose_y = float(row["pose_y"])
    yaw = float(row["pose_yaw"]) + yaw_offset

    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.18)

    map_masked = np.ma.masked_where(M == 0, M)

    map_cmap = cm.get_cmap(cmap_name).copy()
    map_cmap.set_bad(color=zero_color)

    valid = M[M != 0]
    vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
    vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

    image = ax.imshow(
        map_masked,
        origin="lower",
        aspect="equal",
        cmap=map_cmap,
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
    )

    image.set_extent([0, width, 0, height])

    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label("M")

    pose_scatter = ax.scatter(
        [pose_x],
        [pose_y],
        s=80,
        marker="x",
        color=pose_color,
        label="Pose",
    )

    arrow_len = 2.0
    dx = arrow_len * np.cos(yaw)
    dy = arrow_len * np.sin(yaw)
    arrow = ax.arrow(
        pose_x,
        pose_y,
        dx,
        dy,
        head_width=2.00,
        head_length=2.00,
        length_includes_head=True,
        color=arrow_color,
    )

    ax.set_title(f"Map at t={t_rel:.6f}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(False)
    ax.legend()

    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.04])
    slider = Slider(
        ax=ax_slider,
        label="t [s]",
        valmin=float(map_df["t"].iloc[0]) - t0,
        valmax=float(map_df["t"].iloc[-1]) - t0,
        valinit=t_rel,
    )

    def update(val):
        nonlocal arrow

        t_rel = float(slider.val)
        t_abs = t_rel + t0

        idx = get_closest_sample_idx(map_df, t_abs)
        row = map_df.loc[idx]

        width = int(row["width"])
        height = int(row["height"])
        M = parse_map_string(row["map"], width, height)

        pose_x = float(row["pose_x"])
        pose_y = float(row["pose_y"])
        yaw = float(row["pose_yaw"]) + yaw_offset

        map_masked = np.ma.masked_where(M == 0, M)
        image.set_data(map_masked)
        image.set_extent([0, width, 0, height])

        valid = M[M != 0]
        if len(valid) > 0:
            image.set_clim(
                vmin=float(np.percentile(valid, 0.05)),
                vmax=float(np.percentile(valid, 99)),
            )
        else:
            image.set_clim(vmin=0, vmax=1)

        pose_scatter.set_offsets(np.array([[pose_x, pose_y]]))

        arrow.remove()
        dx = arrow_len * np.cos(yaw)
        dy = arrow_len * np.sin(yaw)
        arrow = ax.arrow(
            pose_x,
            pose_y,
            dx,
            dy,
            head_width=0.35,
            head_length=0.55,
            length_includes_head=True,
            color=arrow_color,
        )

        ax.set_title(f"Map at t={t_rel:.6f}")
        fig.canvas.draw_idle()

    slider.on_changed(update)

    fig.suptitle(title)
    plt.show()
