import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib import cm
import json



# CONFIG EXTRACTOR ----------
def load_local_map_generator_params(config_path):
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)

    params = data["/**"]["ros__parameters"]["local_map_generator"]

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
