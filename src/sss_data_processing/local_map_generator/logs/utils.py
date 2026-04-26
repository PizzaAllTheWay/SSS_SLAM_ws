import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib import cm
import json
import matplotlib.patches as patches
import colorsys
from matplotlib.textpath import TextPath
from matplotlib.patches import PathPatch, Rectangle



# CONFIG EXTRACTOR ----------
def load_local_map_generator_params(config_path):
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)

    params = data["/**"]["ros__parimport colorsysameters"]["local_map_generator"]

    return {
        "max_range": params["max_range"],
        "port_scale": params["transducers"]["port"]["blind_zone_scale"],
        "stb_scale": params["transducers"]["starboard"]["blind_zone_scale"],
    }

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
    zero_hold_threshold=None,
):
    df = df.copy().sort_values("t").reset_index(drop=True)
    df = df[(df["t"] >= t_start) & (df["t"] <= t_stop)].reset_index(drop=True)

    if df.empty:
        raise ValueError("No performance samples inside [t_start, t_stop].")

    if zero_dt is None:
        dt = np.diff(df["t"].values)
        dt_valid = dt[dt > 0]
        zero_dt = np.median(dt_valid) if len(dt_valid) > 0 else 0.1

    fill_threshold = gap_threshold if zero_hold_threshold is None else zero_hold_threshold

    rows = []
    dropout_spans = []

    def add_zero_rows(t0, t1, include_start, include_stop):
        start_i = 0 if include_start else 1
        stop_extra = 1 if include_stop else 0

        n_fill = int(np.floor((t1 - t0) / zero_dt)) - 1 + stop_extra

        if n_fill <= 0:
            return

        fill_times = t0 + zero_dt * np.arange(start_i, n_fill + 1)

        for tf in fill_times:
            if tf <= t0 and not include_start:
                continue
            if tf >= t1 and not include_stop:
                break
            if tf > t1:
                break

            rows.append({
                "t": tf,
                "runtime_s": 0.0,
                "cpu_percent": 0.0,
                "ram_mb": 0.0,
                "is_artificial": True,
            })

    first_t = df["t"].iloc[0]
    first_gap = first_t - t_start

    if first_gap > gap_threshold:
        dropout_spans.append((t_start, first_t))

    if first_gap > fill_threshold:
        add_zero_rows(t_start, first_t, include_start=True, include_stop=False)

    for i in range(len(df) - 1):
        row = df.iloc[i].to_dict()
        row["is_artificial"] = False
        rows.append(row)

        t0 = df.iloc[i]["t"]
        t1 = df.iloc[i + 1]["t"]
        dt_gap = t1 - t0

        if dt_gap > gap_threshold:
            dropout_spans.append((t0, t1))

        if dt_gap > fill_threshold:
            add_zero_rows(t0, t1, include_start=False, include_stop=False)

    last_row = df.iloc[-1].to_dict()
    last_row["is_artificial"] = False
    rows.append(last_row)

    last_t = df["t"].iloc[-1]
    last_gap = t_stop - last_t

    if last_gap > gap_threshold:
        dropout_spans.append((last_t, t_stop))

    if last_gap > fill_threshold:
        add_zero_rows(last_t, t_stop, include_start=False, include_stop=True)

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

# GEOMETRIC TIMELINE ----------
def build_geometric_correction_timeline(
    altitude_df,
    geometric_df,
    t_start,
    t_stop,
    gap_threshold=5.0,
    zero_dt=None,
):
    altitude_df = altitude_df.copy().sort_values("t").reset_index(drop=True)
    geometric_df = geometric_df.copy().sort_values("t").reset_index(drop=True)

    altitude_df = altitude_df[(altitude_df["t"] >= t_start) & (altitude_df["t"] <= t_stop)].reset_index(drop=True)
    geometric_df = geometric_df[(geometric_df["t"] >= t_start) & (geometric_df["t"] <= t_stop)].reset_index(drop=True)

    if altitude_df.empty and geometric_df.empty:
        raise ValueError("No geometric correction samples inside [t_start, t_stop].")

    t_all = np.concatenate([
        altitude_df["t"].values if not altitude_df.empty else np.array([]),
        geometric_df["t"].values if not geometric_df.empty else np.array([]),
    ])
    t_all = np.sort(t_all)

    if zero_dt is None:
        dt = np.diff(t_all)
        dt_valid = dt[dt > 0]
        zero_dt = np.median(dt_valid) if len(dt_valid) > 0 else 0.1

    alt = altitude_df[["t", "h_dvl"]].copy()
    geo = geometric_df[["t", "h_port", "h_stb", "r_fbr_port", "r_fbr_stb"]].copy()

    if not alt.empty and not geo.empty:
        df = pd.merge_asof(
            alt.sort_values("t"),
            geo.sort_values("t"),
            on="t",
            direction="nearest",
            tolerance=zero_dt * 0.5,
        )
    elif not alt.empty:
        df = alt.copy()
        df["h_port"] = np.nan
        df["h_stb"] = np.nan
        df["r_fbr_port"] = np.nan
        df["r_fbr_stb"] = np.nan
    else:
        df = geo.copy()
        df["h_dvl"] = np.nan

    df = df.sort_values("t").reset_index(drop=True)

    rows = []
    dropout_spans = []

    first_t = df["t"].iloc[0]
    if first_t - t_start > gap_threshold:
        dropout_spans.append((t_start, first_t))
        n_fill = int(np.floor((first_t - t_start) / zero_dt))
        if n_fill > 0:
            for tf in t_start + zero_dt * np.arange(0, n_fill + 1):
                if tf >= first_t:
                    break
                rows.append({
                    "t": tf,
                    "h_dvl": 0.0,
                    "h_port": 0.0,
                    "h_stb": 0.0,
                    "r_fbr_port": 0.0,
                    "r_fbr_stb": 0.0,
                    "is_artificial": True,
                })

    for i in range(len(df) - 1):
        row = df.iloc[i].to_dict()
        row["is_artificial"] = False
        rows.append(row)

        t0 = df.iloc[i]["t"]
        t1 = df.iloc[i + 1]["t"]

        if t1 - t0 > gap_threshold:
            dropout_spans.append((t0, t1))
            n_fill = int(np.floor((t1 - t0) / zero_dt)) - 1
            if n_fill > 0:
                for tf in t0 + zero_dt * np.arange(1, n_fill + 1):
                    if tf >= t1:
                        break
                    rows.append({
                        "t": tf,
                        "h_dvl": 0.0,
                        "h_port": 0.0,
                        "h_stb": 0.0,
                        "r_fbr_port": 0.0,
                        "r_fbr_stb": 0.0,
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
            for tf in last_t + zero_dt * np.arange(1, n_fill + 1):
                if tf > t_stop:
                    break
                rows.append({
                    "t": tf,
                    "h_dvl": 0.0,
                    "h_port": 0.0,
                    "h_stb": 0.0,
                    "r_fbr_port": 0.0,
                    "r_fbr_stb": 0.0,
                    "is_artificial": True,
                })

    out = pd.DataFrame(rows).sort_values("t").reset_index(drop=True)
    out["t_rel"] = out["t"] - t_start

    return out, dropout_spans, zero_dt

# SWATH + GEOMETRIC TIMELINE ----------
def build_swath_geometric_timeline(
    raw_df,
    geometric_df,
    t_start,
    t_stop,
    gap_threshold=5.0,
    zero_dt=None,
):
    raw_df = raw_df.copy().sort_values("t").reset_index(drop=True)
    geometric_df = geometric_df.copy().sort_values("t").reset_index(drop=True)

    raw_df = raw_df[(raw_df["t"] >= t_start) & (raw_df["t"] <= t_stop)].reset_index(drop=True)
    geometric_df = geometric_df[(geometric_df["t"] >= t_start) & (geometric_df["t"] <= t_stop)].reset_index(drop=True)

    if raw_df.empty:
        raise ValueError("No raw swath samples inside [t_start, t_stop].")

    if zero_dt is None:
        dt = np.diff(raw_df["t"].values)
        dt_valid = dt[dt > 0]
        zero_dt = np.median(dt_valid) if len(dt_valid) > 0 else 0.1

    merged = pd.merge_asof(
        raw_df[["t", "port", "starboard"]].sort_values("t"),
        geometric_df[["t", "r_fbr_port", "r_fbr_stb"]].sort_values("t"),
        on="t",
        direction="nearest",
        tolerance=zero_dt * 0.5,
    )

    rows = []
    dropout_spans = []

    first_t = merged["t"].iloc[0]
    if first_t - t_start > gap_threshold:
        dropout_spans.append((t_start, first_t))

        n_fill = int(np.floor((first_t - t_start) / zero_dt))
        if n_fill > 0:
            for tf in t_start + zero_dt * np.arange(0, n_fill + 1):
                if tf >= first_t:
                    break
                rows.append({
                    "t": tf,
                    "port": "",
                    "starboard": "",
                    "r_fbr_port": 0.0,
                    "r_fbr_stb": 0.0,
                    "is_artificial": True,
                })

    for i in range(len(merged) - 1):
        row = merged.iloc[i].to_dict()
        row["is_artificial"] = False
        rows.append(row)

        t0 = merged.iloc[i]["t"]
        t1 = merged.iloc[i + 1]["t"]
        dt_gap = t1 - t0

        if dt_gap > gap_threshold:
            dropout_spans.append((t0, t1))

            n_fill = int(np.floor(dt_gap / zero_dt)) - 1
            if n_fill > 0:
                for tf in t0 + zero_dt * np.arange(1, n_fill + 1):
                    if tf >= t1:
                        break
                    rows.append({
                        "t": tf,
                        "port": "",
                        "starboard": "",
                        "r_fbr_port": 0.0,
                        "r_fbr_stb": 0.0,
                        "is_artificial": True,
                    })

    last_row = merged.iloc[-1].to_dict()
    last_row["is_artificial"] = False
    rows.append(last_row)

    last_t = merged["t"].iloc[-1]
    if t_stop - last_t > gap_threshold:
        dropout_spans.append((last_t, t_stop))

        n_fill = int(np.floor((t_stop - last_t) / zero_dt))
        if n_fill > 0:
            for tf in last_t + zero_dt * np.arange(1, n_fill + 1):
                if tf > t_stop:
                    break
                rows.append({
                    "t": tf,
                    "port": "",
                    "starboard": "",
                    "r_fbr_port": 0.0,
                    "r_fbr_stb": 0.0,
                    "is_artificial": True,
                })

    out = pd.DataFrame(rows)
    out = out.sort_values("t").reset_index(drop=True)
    out["t_rel"] = out["t"] - t_start

    return out, dropout_spans, zero_dt

def _range_to_bin(r, scale, max_range, samples_per_beam):
    r_scaled = np.asarray(r) * scale
    slant_resolution = max_range / samples_per_beam
    return np.floor(r_scaled / slant_resolution).clip(0, samples_per_beam - 1).astype(int)

# GEOMETRIC CORRECTION PLOT ----------
def plot_geometric_correction(
    altitude_df,
    geometric_df,
    raw_df,
    t_start,
    t_stop,
    max_range,
    port_scale,
    stb_scale,
    title="Geometric Correction",
    gap_threshold=5.0,
):
    # Top plot: heights with zero-filled dropouts
    df_h, dropout_spans, zero_dt = build_geometric_correction_timeline(
        altitude_df=altitude_df,
        geometric_df=geometric_df,
        t_start=t_start,
        t_stop=t_stop,
        gap_threshold=gap_threshold,
        zero_dt=None,
    )

    # Bottom plot: raw swath with r_fbr overlays and zero-filled dropouts
    df_swath, _, _ = build_swath_geometric_timeline(
        raw_df=raw_df,
        geometric_df=geometric_df,
        t_start=t_start,
        t_stop=t_stop,
        gap_threshold=gap_threshold,
        zero_dt=zero_dt,
    )

    raw_port = parse_array(df_swath["port"])
    raw_stb = parse_array(df_swath["starboard"])

    raw_disp = np.hstack([raw_port, raw_stb])

    n = raw_port.shape[1]
    if n == 0:
        raise ValueError("Parsed raw swath has zero width.")

    bin_port = _range_to_bin(df_swath["r_fbr_port"].fillna(0.0).values, port_scale, max_range, n)
    bin_stb = _range_to_bin(df_swath["r_fbr_stb"].fillna(0.0).values, stb_scale, max_range, n)

    x_time = df_swath["t_rel"].values
    dt = zero_dt if zero_dt is not None else 0.1
    t0_img = x_time[0] - 0.5 * dt
    t1_img = x_time[-1] + 0.5 * dt

    # Port overlay goes on left half, starboard on right half
    y_port = (n - 1) - bin_port
    y_stb = n + bin_stb

    fig, axes = create_stacked_plot(
        2,
        title=title,
        xlabel="Time since t_start [s]",
        ylabels=["Height / Altitude [m]", "Across-track samples"],
        sharex=True,
    )

    # Top
    add_series(axes[0], df_h["t_rel"], df_h["h_dvl"], label="h_dvl")
    add_series(axes[0], df_h["t_rel"], df_h["h_port"], label="h_port")
    add_series(axes[0], df_h["t_rel"], df_h["h_stb"], label="h_stb")
    add_dropout_spans(axes[0], dropout_spans, t_ref=t_start)

    # Bottom: transpose so time is x-axis
    axes[1].imshow(
        raw_disp.T,
        aspect="auto",
        cmap="copper",
        origin="upper",
        extent=[t0_img, t1_img, 2 * n, 0],
    )
    axes[1].plot(x_time, y_port, linewidth=1.2, label="r_fbr_port")
    axes[1].plot(x_time, y_stb, linewidth=1.2, label="r_fbr_stb")
    axes[1].set_ylabel("Across-track samples")
    axes[1].set_title("Raw swath with first-bottom-return overlay")

    finalize_plot()

# MAP HELPERS ----------
def get_closest_pose_row(pose_df, t_target):
    idx = (pose_df["t"] - t_target).abs().idxmin()
    return pose_df.loc[idx]

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

def get_closest_sample_idx(df, t_target):
    return (df["t"] - t_target).abs().idxmin()

# FULL MAP MOSAIC ----------
def build_and_save_full_map_mosaic(
    map_df,
    map_origin_df,
    map_pose_df,
    out_dir,
    mosaic_name="full_map_mosaic",
    stride=1,
    pixel_stride=1,
):
    os.makedirs(out_dir, exist_ok=True)

    # ---------- pass 1: global bounds ----------
    x_mins = []
    x_maxs = []
    y_mins = []
    y_maxs = []

    for _, map_row in map_df.iloc[::stride].iterrows():
        t_abs = float(map_row["t"])

        origin_row = get_closest_pose_row(map_origin_df, t_abs)
        pose_row = get_closest_pose_row(map_pose_df, t_abs)

        width = int(map_row["width"])
        height = int(map_row["height"])
        resolution = float(map_row["resolution"]) * pixel_stride

        if width <= 0 or height <= 0:
            continue

        width = int(np.ceil(width / pixel_stride))
        height = int(np.ceil(height / pixel_stride))

        origin_x_px = float(origin_row["x"]) / pixel_stride
        origin_y_px = float(origin_row["y"]) / pixel_stride

        pose_x = float(pose_row["x"])
        pose_y = float(pose_row["y"])

        x_min = pose_x - origin_x_px * resolution
        x_max = x_min + width * resolution
        y_min = pose_y - origin_y_px * resolution
        y_max = y_min + height * resolution

        x_mins.append(x_min)
        x_maxs.append(x_max)
        y_mins.append(y_min)
        y_maxs.append(y_max)

    if len(x_mins) == 0:
        raise ValueError("No valid maps found")

    global_x_min = min(x_mins)
    global_x_max = max(x_maxs)
    global_y_min = min(y_mins)
    global_y_max = max(y_maxs)

    base_resolution = float(map_df.iloc[0]["resolution"]) * pixel_stride

    mosaic_width = int(np.ceil((global_x_max - global_x_min) / base_resolution))
    mosaic_height = int(np.ceil((global_y_max - global_y_min) / base_resolution))

    mmap_path = os.path.join(out_dir, f"{mosaic_name}.dat")
    meta_path = os.path.join(out_dir, f"{mosaic_name}.json")

    mosaic = np.memmap(
        mmap_path,
        dtype=np.uint8,
        mode="w+",
        shape=(mosaic_height, mosaic_width),
    )
    mosaic[:] = 0

    last_pose = None
    last_origin = None
    last_t = None

    # ---------- pass 2: stamp maps ----------
    for _, map_row in map_df.iloc[::stride].iterrows():
        t_abs = float(map_row["t"])
        last_t = t_abs

        origin_row = get_closest_pose_row(map_origin_df, t_abs)
        pose_row = get_closest_pose_row(map_pose_df, t_abs)

        width = int(map_row["width"])
        height = int(map_row["height"])
        resolution = float(map_row["resolution"])

        if width <= 0 or height <= 0:
            continue

        M = parse_map_string(map_row["map"], width, height)
        if M.size == 0:
            continue

        if pixel_stride > 1:
            M = M[::pixel_stride, ::pixel_stride]

        height, width = M.shape
        resolution = resolution * pixel_stride

        origin_x_px = float(origin_row["x"]) / pixel_stride
        origin_y_px = float(origin_row["y"]) / pixel_stride

        pose_x = float(pose_row["x"])
        pose_y = float(pose_row["y"])
        pose_yaw = float(pose_row["yaw"])

        x_min = pose_x - origin_x_px * resolution
        y_min = pose_y - origin_y_px * resolution

        col0 = int(round((x_min - global_x_min) / resolution))
        row0 = int(round((y_min - global_y_min) / resolution))
        col1 = col0 + width
        row1 = row0 + height

        if row0 < 0 or col0 < 0 or row1 > mosaic_height or col1 > mosaic_width:
            continue

        target = mosaic[row0:row1, col0:col1]

        mask = M != 0
        target[mask] = M[mask]

        last_pose = {
            "x": pose_x,
            "y": pose_y,
            "yaw": pose_yaw,
        }
        last_origin = {
            "x": x_min,
            "y": y_min,
        }

    mosaic.flush()

    metadata = {
        "mmap_path": mmap_path,
        "dtype": "uint8",
        "shape": [mosaic_height, mosaic_width],
        "resolution": base_resolution,
        "global_x_min": global_x_min,
        "global_y_min": global_y_min,
        "global_x_max": global_x_max,
        "global_y_max": global_y_max,
        "last_pose": last_pose,
        "last_origin": last_origin,
        "last_t": last_t,
        "stride": stride,
        "pixel_stride": pixel_stride,
    }

    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)

    return mmap_path, meta_path


def load_full_map_mosaic(meta_path):
    with open(meta_path, "r") as f:
        meta = json.load(f)

    mosaic = np.memmap(
        meta["mmap_path"],
        dtype=np.uint8,
        mode="r",
        shape=tuple(meta["shape"]),
    )

    return mosaic, meta


def plot_saved_full_map_mosaic(
    meta_path,
    title="Full Map Mosaic",
    xlabel="x [m]",
    ylabel="y [m]",
    cmap_name="copper",
    zero_color="white",
    yaw_offset=0.0,
):
    mosaic, meta = load_full_map_mosaic(meta_path)

    fig, ax = plt.subplots(figsize=(12, 10))

    mosaic_masked = np.ma.masked_where(mosaic == 0, mosaic)

    map_cmap = cm.get_cmap(cmap_name).copy()
    map_cmap.set_bad(color=zero_color)

    valid = mosaic[mosaic != 0]
    vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
    vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

    ax.imshow(
        mosaic_masked,
        origin="lower",
        aspect="equal",
        cmap=map_cmap,
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
        extent=[
            meta["global_x_min"],
            meta["global_x_max"],
            meta["global_y_min"],
            meta["global_y_max"],
        ],
    )

    if meta["last_pose"] is not None:
        pose_x = meta["last_pose"]["x"]
        pose_y = meta["last_pose"]["y"]
        pose_yaw = meta["last_pose"]["yaw"] + yaw_offset

        ax.scatter(
            [pose_x],
            [pose_y],
            s=90,
            marker="x",
            color="orange",
            label="Latest pose",
        )

        arrow_len = 1.5
        dx = arrow_len * np.cos(pose_yaw)
        dy = arrow_len * np.sin(pose_yaw)
        ax.arrow(
            pose_x,
            pose_y,
            dx,
            dy,
            head_width=0.20,
            head_length=0.30,
            length_includes_head=True,
            color="black",
        )

    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.axis("equal")
    ax.grid(False)
    ax.legend()
    plt.show()

# LANDMARKS ----------
def filter_time_window(df, t_start=None, t_stop=None):
    df = df.copy().sort_values("t").reset_index(drop=True)

    if t_start is not None:
        df = df[df["t"] > t_start]

    if t_stop is not None:
        df = df[df["t"] <= t_stop]

    return df.reset_index(drop=True)


def build_and_save_partial_map_mosaic_from_time_window(
    map_df,
    map_origin_df,
    map_pose_df,
    out_dir,
    t_start,
    t_stop,
    mosaic_name="mission_partial",
    stride=1,
    pixel_stride=1,
):
    map_df = filter_time_window(map_df, t_start, t_stop)

    if len(map_df) == 0:
        raise ValueError("No map samples inside selected time window")

    return build_and_save_full_map_mosaic(
        map_df=map_df,
        map_origin_df=map_origin_df,
        map_pose_df=map_pose_df,
        out_dir=out_dir,
        mosaic_name=mosaic_name,
        stride=stride,
        pixel_stride=pixel_stride,
    )


def _rot2(yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)

    return np.array([
        [c, -s],
        [s,  c],
    ], dtype=float)


def _polar_measurement_to_pose_xy(r, theta, yaw_offset=0.0):
    angle = theta + yaw_offset

    return np.array([
        r * np.cos(angle),
        r * np.sin(angle),
    ], dtype=float)


def _pose_xy_to_world_xy(pose_x, pose_y, pose_yaw, local_xy):
    pose_xy = np.array([pose_x, pose_y], dtype=float)
    return pose_xy + _rot2(pose_yaw) @ local_xy


def _polar_covariance_to_world_xy(row, yaw_offset=0.0):
    pose_yaw = float(row["pose_yaw"])
    r = float(row["z_r"])
    theta = float(row["z_theta"])

    angle_world = pose_yaw + theta + yaw_offset

    R_polar = np.array([
        [float(row["R_z_rr"]),     float(row["R_z_rtheta"])],
        [float(row["R_z_thetar"]), float(row["R_z_thetatheta"])],
    ], dtype=float)

    J = np.array([
        [np.cos(angle_world), -r * np.sin(angle_world)],
        [np.sin(angle_world),  r * np.cos(angle_world)],
    ], dtype=float)

    return J @ R_polar @ J.T


def _pose_color_from_index(pose_index):
    # Darker, vivid colors only.
    # Avoids white, yellow, light green, and other washed-out tones.
    palette = [
        (0.80, 0.15, 0.15),  # dark red
        (0.15, 0.35, 0.80),  # dark blue
        (0.45, 0.15, 0.75),  # purple
        (0.00, 0.55, 0.65),  # teal
        (0.75, 0.15, 0.55),  # magenta
        (0.80, 0.30, 0.10),  # burnt orange
        (0.25, 0.20, 0.65),  # indigo
        (0.65, 0.10, 0.35),  # wine
        (0.10, 0.45, 0.35),  # dark turquoise
        (0.55, 0.10, 0.10),  # deep red
    ]

    return palette[int(pose_index) % len(palette)]


def prepare_pose_plot_df(feature_pose_df, map_pose_df=None):
    if map_pose_df is not None:
        pose_plot_df = map_pose_df[["t", "x", "y", "yaw"]].copy()
    else:
        pose_plot_df = feature_pose_df[["t", "x", "y", "yaw"]].copy()

    pose_plot_df = pose_plot_df.sort_values("t").reset_index(drop=True)
    pose_plot_df["pose_index"] = np.arange(len(pose_plot_df))

    return pose_plot_df


def attach_map_origin_and_pose_to_landmarks(matched_df, map_origin_df, map_pose_df):
    if map_origin_df is None or map_pose_df is None:
        return matched_df

    out = matched_df.copy().sort_values("t").reset_index(drop=True)

    origin_df = (
        map_origin_df[["t", "x", "y"]]
        .copy()
        .sort_values("t")
        .rename(columns={"x": "origin_x_px", "y": "origin_y_px"})
        .reset_index(drop=True)
    )

    map_pose_small_df = (
        map_pose_df[["t", "x", "y", "yaw"]]
        .copy()
        .sort_values("t")
        .rename(columns={"x": "map_pose_x", "y": "map_pose_y", "yaw": "map_pose_yaw"})
        .reset_index(drop=True)
    )

    out = pd.merge_asof(
        out.sort_values("t"),
        origin_df.sort_values("t"),
        on="t",
        direction="nearest",
    )

    out = pd.merge_asof(
        out.sort_values("t"),
        map_pose_small_df.sort_values("t"),
        on="t",
        direction="nearest",
    )

    return out.dropna(
        subset=["origin_x_px", "origin_y_px", "map_pose_x", "map_pose_y"]
    ).reset_index(drop=True)


def match_landmarks_to_feature_poses(
    landmark_df,
    feature_pose_df,
    tolerance=None,
    time_decimals=6,
):
    landmark_df = landmark_df.copy().sort_values("t").reset_index(drop=True)
    feature_pose_df = feature_pose_df.copy().sort_values("t").reset_index(drop=True)

    if "pose_index" not in feature_pose_df.columns:
        feature_pose_df["pose_index"] = np.arange(len(feature_pose_df))

    required_pose_cols = ["t", "x", "y", "yaw", "pose_index"]
    missing = [c for c in required_pose_cols if c not in feature_pose_df.columns]
    if missing:
        raise ValueError(f"feature_pose_df missing columns: {missing}")

    keep_cols = [
        "t",
        "x",
        "y",
        "yaw",
        "pose_index",
        "origin_x_px",
        "origin_y_px",
        "map_pose_x",
        "map_pose_y",
        "map_pose_yaw",
    ]
    keep_cols = [c for c in keep_cols if c in feature_pose_df.columns]

    pose_df = feature_pose_df[keep_cols].copy().rename(columns={
        "x": "pose_x",
        "y": "pose_y",
        "yaw": "pose_yaw",
    })

    landmark_df["t_key"] = landmark_df["t"].round(time_decimals)
    pose_df["t_key"] = pose_df["t"].round(time_decimals)

    matched = landmark_df.merge(
        pose_df.drop(columns=["t"]),
        on="t_key",
        how="left",
    ).drop(columns=["t_key"])

    missing_mask = matched[["pose_x", "pose_y", "pose_yaw", "pose_index"]].isna().any(axis=1)

    if missing_mask.any():
        if tolerance is None:
            dt = np.diff(feature_pose_df["t"].to_numpy(dtype=float))
            dt_valid = dt[dt > 0]
            tolerance = 0.5 * np.median(dt_valid) if len(dt_valid) > 0 else 0.05

        unmatched = landmark_df.loc[missing_mask].drop(columns=["t_key"]).copy()

        fallback = pd.merge_asof(
            unmatched.sort_values("t"),
            pose_df.drop(columns=["t_key"]).sort_values("t"),
            on="t",
            direction="nearest",
            tolerance=tolerance,
        )

        fill_cols = [c for c in fallback.columns if c != "t" and c in matched.columns]
        matched.loc[missing_mask, fill_cols] = fallback[fill_cols].to_numpy()

    return matched.dropna(
        subset=["pose_x", "pose_y", "pose_yaw", "pose_index"]
    ).reset_index(drop=True)


def landmark_measurement_to_world(row, yaw_offset=0.0, resolution=None):
    if (
        resolution is not None
        and "cx" in row
        and "cy" in row
        and "origin_x_px" in row
        and "origin_y_px" in row
        and "map_pose_x" in row
        and "map_pose_y" in row
        and not pd.isna(row["origin_x_px"])
        and not pd.isna(row["origin_y_px"])
        and not pd.isna(row["map_pose_x"])
        and not pd.isna(row["map_pose_y"])
    ):
        lm_x = float(row["map_pose_x"]) + (float(row["cx"]) - float(row["origin_x_px"])) * float(resolution)
        lm_y = float(row["map_pose_y"]) + (float(row["cy"]) - float(row["origin_y_px"])) * float(resolution)
        return lm_x, lm_y

    pose_x = float(row["pose_x"])
    pose_y = float(row["pose_y"])
    pose_yaw = float(row["pose_yaw"])

    r = float(row["z_r"])
    theta = float(row["z_theta"])

    local_xy = _polar_measurement_to_pose_xy(r, theta, yaw_offset)
    world_xy = _pose_xy_to_world_xy(pose_x, pose_y, pose_yaw, local_xy)

    return float(world_xy[0]), float(world_xy[1])


def _draw_world_landmark_measurement_line(
    ax,
    row,
    yaw_offset=0.0,
    resolution=None,
    linewidth=0.9,
    alpha=0.85,
):
    pose_x = float(row["pose_x"])
    pose_y = float(row["pose_y"])

    lm_x, lm_y = landmark_measurement_to_world(
        row,
        yaw_offset=yaw_offset,
        resolution=resolution,
    )

    color = _pose_color_from_index(row["pose_index"])

    ax.plot(
        [pose_x, lm_x],
        [pose_y, lm_y],
        linewidth=linewidth,
        color=color,
        alpha=alpha,
        zorder=3,
    )

    return lm_x, lm_y, color


def _draw_world_landmark_covariance_ellipse(
    ax,
    row,
    yaw_offset=0.0,
    resolution=None,
    n_sigma=2.0,
    color=None,
):
    lm_x, lm_y = landmark_measurement_to_world(
        row,
        yaw_offset=yaw_offset,
        resolution=resolution,
    )

    if (
        resolution is not None
        and "origin_x_px" in row
        and "origin_y_px" in row
        and "map_pose_x" in row
        and "map_pose_y" in row
        and not pd.isna(row["origin_x_px"])
        and not pd.isna(row["origin_y_px"])
    ):
        pose_x = float(row["pose_x"])
        pose_y = float(row["pose_y"])

        dx = lm_x - pose_x
        dy = lm_y - pose_y

        r = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx)

        R_polar = np.array([
            [float(row["R_z_rr"]),     float(row["R_z_rtheta"])],
            [float(row["R_z_thetar"]), float(row["R_z_thetatheta"])],
        ], dtype=float)

        J = np.array([
            [np.cos(theta), -r * np.sin(theta)],
            [np.sin(theta),  r * np.cos(theta)],
        ], dtype=float)

        R_xy = J @ R_polar @ J.T
    else:
        R_xy = _polar_covariance_to_world_xy(row, yaw_offset=yaw_offset)

    vals, vecs = np.linalg.eigh(R_xy)
    vals = np.maximum(vals, 0.0)

    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    angle_deg = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    if color is None:
        color = _pose_color_from_index(row["pose_index"])

    ellipse = patches.Ellipse(
        (lm_x, lm_y),
        width=2.0 * n_sigma * np.sqrt(vals[0]),
        height=2.0 * n_sigma * np.sqrt(vals[1]),
        angle=angle_deg,
        facecolor=(*color[:3], 0.10),
        edgecolor=(*color[:3], 0.55),
        linewidth=1.0,
        zorder=2,
    )

    ax.add_patch(ellipse)


def _plot_mosaic_background(ax, mosaic, meta, cmap_name="copper", zero_color="white"):
    mosaic_masked = np.ma.masked_where(mosaic == 0, mosaic)

    map_cmap = cm.get_cmap(cmap_name).copy()
    map_cmap.set_bad(color=zero_color)

    valid = mosaic[mosaic != 0]
    vmin = np.percentile(valid, 0.05) if len(valid) > 0 else 0
    vmax = np.percentile(valid, 99) if len(valid) > 0 else 1

    ax.imshow(
        mosaic_masked,
        origin="lower",
        aspect="equal",
        cmap=map_cmap,
        interpolation="nearest",
        vmin=vmin,
        vmax=vmax,
        extent=[
            meta["global_x_min"],
            meta["global_x_max"],
            meta["global_y_min"],
            meta["global_y_max"],
        ],
    )

def _plot_feature_poses(
    ax,
    pose_plot_df,
    yaw_offset=0.0,
    draw_pose_heading=True,
    path_linewidth=1.0,
    pose_size=20,
    arrow_len=1.0,
    arrow_width=0.002,
):
    ax.plot(
        pose_plot_df["x"],
        pose_plot_df["y"],
        linewidth=path_linewidth,
        color="black",
        alpha=0.85,
        label="Robot pose path",
        zorder=5,
    )

    ax.scatter(
        pose_plot_df["x"],
        pose_plot_df["y"],
        s=pose_size,
        color="black",
        alpha=0.9,
        label="Robot poses",
        zorder=6,
    )

    if not draw_pose_heading:
        return

    yaw = pose_plot_df["yaw"].to_numpy(dtype=float) + yaw_offset

    ax.quiver(
        pose_plot_df["x"].to_numpy(dtype=float),
        pose_plot_df["y"].to_numpy(dtype=float),
        arrow_len * np.cos(yaw),
        arrow_len * np.sin(yaw),
        angles="xy",
        scale_units="xy",
        scale=1.0,
        width=arrow_width,
        color="black",
        alpha=0.75,
        zorder=7,
    )

def _make_zoom_scaled_text(ax, x, y, label, color, base_fontsize=7, min_fontsize=6, max_fontsize=28):
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ref_span = max(abs(x1 - x0), abs(y1 - y0))

    text = ax.annotate(
        label,
        xy=(x, y),
        xytext=(3, 3),
        textcoords="offset points",
        fontsize=base_fontsize,
        color=color,
        zorder=9,
        bbox=dict(facecolor="white", edgecolor="none", alpha=0.75, pad=1.0),
    )

    text._base_fontsize = base_fontsize
    text._ref_span = ref_span
    text._min_fontsize = min_fontsize
    text._max_fontsize = max_fontsize

    return text


def _draw_data_scaled_text(
    ax,
    x,
    y,
    label,
    color,
    text_size=0.08,
    x_offset=0.05,
    y_offset=0.05,
    bg_pad=0.025,
    z_base=20,
):
    tx = x + x_offset
    ty = y + y_offset

    path = TextPath((tx, ty), label, size=text_size)
    bbox = path.get_extents()

    bg = Rectangle(
        (bbox.x0 - bg_pad, bbox.y0 - bg_pad),
        bbox.width + 2.0 * bg_pad,
        bbox.height + 2.0 * bg_pad,
        facecolor="white",
        edgecolor="none",
        alpha=1.0,
        zorder=z_base,
    )
    ax.add_patch(bg)

    patch = PathPatch(
        path,
        facecolor=color,
        edgecolor="none",
        zorder=z_base + 0.1,
    )
    ax.add_patch(patch)

    return patch

def _plot_landmarks(
    ax,
    matched_df,
    yaw_offset=0.0,
    resolution=None,
    draw_covariance=True,
    label_text_size=0.08,
):
    text_patches = []

    for i, (_, row) in enumerate(matched_df.iterrows()):
        lm_x, lm_y, color = _draw_world_landmark_measurement_line(
            ax,
            row,
            yaw_offset=yaw_offset,
            resolution=resolution,
            linewidth=0.9,
            alpha=0.8,
        )

        if draw_covariance:
            _draw_world_landmark_covariance_ellipse(
                ax,
                row,
                yaw_offset=yaw_offset,
                resolution=resolution,
                n_sigma=2.0,
                color=color,
            )

        ax.scatter(
            lm_x,
            lm_y,
            s=20,
            color=color,
            edgecolors="black",
            linewidths=0.4,
            zorder=8,
        )

        h_hat = float(row["height_value"]) if "height_value" in row and not pd.isna(row["height_value"]) else None
        h_std = float(row["height_std"]) if "height_std" in row and not pd.isna(row["height_std"]) else None

        if h_hat is not None and h_std is not None:
            label = f"ĥ = {h_hat:.2f} ± {h_std:.2f} m"
        elif h_hat is not None:
            label = f"ĥ = {h_hat:.2f} m"
        else:
            label = f"{int(row['label_id'])}"

        z_base = 20 + 2 * i

        text_patch = _draw_data_scaled_text(
            ax,
            lm_x,
            lm_y,
            label,
            color,
            text_size=label_text_size,
            x_offset=0.05,
            y_offset=0.05,
            bg_pad=0.025,
            z_base=z_base,
        )

        text_patches.append(text_patch)

    return text_patches


def plot_saved_partial_mosaic_with_feature_poses_and_landmarks(
    meta_path,
    feature_pose_df,
    landmark_df,
    map_origin_df=None,
    map_pose_df=None,
    t_start=None,
    t_stop=None,
    title="Partial Map with Feature Poses and Landmarks",
    yaw_offset=0.0,
    cmap_name="copper",
    zero_color="white",
    draw_pose_heading=True,
    draw_covariance=True,
    label_text_size=0.6,
):
    feature_pose_df = filter_time_window(feature_pose_df, t_start, t_stop)
    landmark_df = filter_time_window(landmark_df, t_start, t_stop)

    if len(feature_pose_df) == 0:
        raise ValueError("No feature pose samples inside selected time window")
    if len(landmark_df) == 0:
        raise ValueError("No landmark samples inside selected time window")

    mosaic, meta = load_full_map_mosaic(meta_path)
    resolution = float(meta.get("resolution", meta.get("map_resolution", 1.0)))

    if map_origin_df is not None:
        map_origin_df = filter_time_window(map_origin_df, t_start, t_stop)

    if map_pose_df is not None:
        map_pose_df = filter_time_window(map_pose_df, t_start, t_stop)

    pose_plot_df = prepare_pose_plot_df(
        feature_pose_df=feature_pose_df,
        map_pose_df=map_pose_df,
    )

    matched_df = match_landmarks_to_feature_poses(
        landmark_df=landmark_df,
        feature_pose_df=pose_plot_df,
    )

    matched_df = attach_map_origin_and_pose_to_landmarks(
        matched_df=matched_df,
        map_origin_df=map_origin_df,
        map_pose_df=map_pose_df,
    )

    fig, ax = plt.subplots(figsize=(12, 10))

    _plot_mosaic_background(
        ax,
        mosaic=mosaic,
        meta=meta,
        cmap_name=cmap_name,
        zero_color=zero_color,
    )

    _plot_feature_poses(
        ax,
        pose_plot_df=pose_plot_df,
        yaw_offset=yaw_offset,
        draw_pose_heading=draw_pose_heading,
    )

    _plot_landmarks(
        ax,
        matched_df=matched_df,
        yaw_offset=yaw_offset,
        resolution=resolution,
        draw_covariance=draw_covariance,
        label_text_size=label_text_size,
    )

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(False)

    plt.tight_layout()

    plt.show()
