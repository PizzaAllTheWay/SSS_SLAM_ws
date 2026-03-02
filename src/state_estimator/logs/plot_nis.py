import os
import numpy as np
from scipy.stats import chi2

from utils import (
    get_newest_file,
    load_csv,
    create_stacked_plot,
    add_series,
    add_nis_consistency_bounds,
    add_cumulative_mean,
    finalize_plot
)

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

def main():
    # =========================================================
    # AHRS NIS
    # =========================================================
    nis_file = get_newest_file(DATA_DIR, "nis_ahrs")
    df = load_csv(nis_file)

    # Convert to relative time (no CSV modification)
    t_ref = df["t"].iloc[0]
    df["t_rel"] = df["t"] - t_ref

    # Ignore first N seconds (using relative time)
    ignore_seconds = 3.0
    df = df[df["t_rel"] > ignore_seconds].reset_index(drop=True)

    dof = 3
    confidence = 0.95

    fig, axes = create_stacked_plot(
        1,
        title="AHRS NIS Consistency Test",
        xlabel="Time [s]",
        ylabels=["NIS"]
    )

    ax = axes[0]

    # Plot NIS
    add_series(
        ax,
        df["t_rel"],
        df["nis"],
        label="NIS AHRS",
        color="blue"
    )

    # Chi-square bounds
    add_nis_consistency_bounds(
        ax,
        dof=dof,
        confidence=confidence
    )

    # Cumulative mean
    add_cumulative_mean(
        ax,
        df["t_rel"],
        df["nis"]
    )

    # Consistency percentage
    nis = df["nis"].values
    lower = chi2.ppf(1 - confidence, dof)
    upper = chi2.ppf(confidence, dof)

    inside = np.logical_and(nis >= lower, nis <= upper)
    percentage_inside = 100.0 * np.sum(inside) / len(nis)

    print(f"AHRS inside 95% NIS bounds: {percentage_inside:.2f}%")

    ax.text(
        0.02,
        0.95,
        f"Inside 95% bounds: {percentage_inside:.2f}%",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    finalize_plot()

    # =========================================================
    # DEPTH NIS
    # =========================================================
    nis_file = get_newest_file(DATA_DIR, "nis_depth")
    df = load_csv(nis_file)

    t_ref = df["t"].iloc[0]
    df["t_rel"] = df["t"] - t_ref
    df = df[df["t_rel"] > ignore_seconds].reset_index(drop=True)

    dof = 1  # depth = scalar
    fig, axes = create_stacked_plot(
        1,
        title="Depth NIS Consistency Test",
        xlabel="Time [s]",
        ylabels=["NIS"]
    )

    ax = axes[0]

    add_series(
        ax,
        df["t_rel"],
        df["nis"],
        label="NIS Depth",
        color="red"
    )

    add_nis_consistency_bounds(
        ax,
        dof=dof,
        confidence=confidence
    )

    add_cumulative_mean(
        ax,
        df["t_rel"],
        df["nis"]
    )

    nis = df["nis"].values
    lower = chi2.ppf(1 - confidence, dof)
    upper = chi2.ppf(confidence, dof)

    inside = np.logical_and(nis >= lower, nis <= upper)
    percentage_inside = 100.0 * np.sum(inside) / len(nis)

    print(f"Depth inside 95% NIS bounds: {percentage_inside:.2f}%")

    ax.text(
        0.02,
        0.95,
        f"Inside 95% bounds: {percentage_inside:.2f}%",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    finalize_plot()

    # =========================================================
    # DVL NIS
    # =========================================================
    nis_file = get_newest_file(DATA_DIR, "nis_dvl")
    df = load_csv(nis_file)

    t_ref = df["t"].iloc[0]
    df["t_rel"] = df["t"] - t_ref
    df = df[df["t_rel"] > ignore_seconds].reset_index(drop=True)

    dof = 3  # DVL measures 3D velocity
    fig, axes = create_stacked_plot(
        1,
        title="DVL NIS Consistency Test",
        xlabel="Time [s]",
        ylabels=["NIS"]
    )

    ax = axes[0]

    add_series(
        ax,
        df["t_rel"],
        df["nis"],
        label="NIS DVL",
        color="green"
    )

    add_nis_consistency_bounds(
        ax,
        dof=dof,
        confidence=confidence
    )

    add_cumulative_mean(
        ax,
        df["t_rel"],
        df["nis"]
    )

    nis = df["nis"].values
    lower = chi2.ppf(1 - confidence, dof)
    upper = chi2.ppf(confidence, dof)

    inside = np.logical_and(nis >= lower, nis <= upper)
    percentage_inside = 100.0 * np.sum(inside) / len(nis)

    print(f"DVL inside 95% NIS bounds: {percentage_inside:.2f}%")

    ax.text(
        0.02,
        0.95,
        f"Inside 95% bounds: {percentage_inside:.2f}%",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    finalize_plot()

    # =========================================================
    # GPS NIS
    # =========================================================
    nis_file = get_newest_file(DATA_DIR, "nis_gps")
    df = load_csv(nis_file)

    t_ref = df["t"].iloc[0]
    df["t_rel"] = df["t"] - t_ref
    df = df[df["t_rel"] > ignore_seconds].reset_index(drop=True)

    dof = 3  # GPS measures 3D position (NED)
    fig, axes = create_stacked_plot(
        1,
        title="GPS NIS Consistency Test",
        xlabel="Time [s]",
        ylabels=["NIS"]
    )

    ax = axes[0]

    add_series(
        ax,
        df["t_rel"],
        df["nis"],
        label="NIS GPS",
        color="purple"
    )

    add_nis_consistency_bounds(
        ax,
        dof=dof,
        confidence=confidence
    )

    add_cumulative_mean(
        ax,
        df["t_rel"],
        df["nis"]
    )

    nis = df["nis"].values
    lower = chi2.ppf(1 - confidence, dof)
    upper = chi2.ppf(confidence, dof)

    inside = np.logical_and(nis >= lower, nis <= upper)
    percentage_inside = 100.0 * np.sum(inside) / len(nis)

    print(f"GPS inside 95% NIS bounds: {percentage_inside:.2f}%")

    ax.text(
        0.02,
        0.95,
        f"Inside 95% bounds: {percentage_inside:.2f}%",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    finalize_plot()

if __name__ == "__main__":
    main()