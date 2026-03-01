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
        df["nis_ahrs"],
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
        df["nis_ahrs"]
    )

    # Consistency percentage
    nis = df["nis_ahrs"].values
    lower = chi2.ppf(1 - confidence, dof)
    upper = chi2.ppf(confidence, dof)

    inside = np.logical_and(nis >= lower, nis <= upper)
    percentage_inside = 100.0 * np.sum(inside) / len(nis)

    print(f"Percentage inside 95% bounds: {percentage_inside:.2f}%")

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