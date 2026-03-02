import os
import numpy as np
import pandas as pd
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


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def main():
    bench_file = get_newest_file(DATA_DIR, "benchmark")
    est_file   = get_newest_file(DATA_DIR, "state_estimate")

    df_bench = load_csv(bench_file)
    df_est   = load_csv(est_file)

    # ---------------- TIME ALIGNMENT ----------------
    t0 = min(df_est["t"].iloc[0], df_bench["t"].iloc[0])
    df_est["t_rel"]   = df_est["t"]   - t0
    df_bench["t_rel"] = df_bench["t"] - t0

    ignore_seconds = 3.0
    df_est   = df_est[df_est["t_rel"] > ignore_seconds].reset_index(drop=True)
    df_bench = df_bench[df_bench["t_rel"] > ignore_seconds].reset_index(drop=True)

    df_est   = df_est.sort_values("t_rel")
    df_bench = df_bench.sort_values("t_rel")

    dt_est = df_est["t_rel"].diff().median()
    tolerance = 2.0 * dt_est

    df = pd.merge_asof(
        df_est,
        df_bench,
        on="t_rel",
        direction="nearest",
        tolerance=tolerance,
        suffixes=("_est", "_bench"),
    ).dropna().reset_index(drop=True)

    # ---------------- ZERO POSITION REFERENCE ----------------
    px0 = df["px_bench"].iloc[0]
    py0 = df["py_bench"].iloc[0]
    pz0 = df["pz_bench"].iloc[0]

    for col in ["px", "py", "pz"]:
        df[f"{col}_bench"] -= df[f"{col}_bench"].iloc[0]

    # ---------------- NEES ----------------
    nees = []

    for k in range(len(df)):

        e = np.array([
            wrap_angle(df["roll_est"].iloc[k]  - df["roll_bench"].iloc[k]),
            wrap_angle(df["pitch_est"].iloc[k] - df["pitch_bench"].iloc[k]),
            wrap_angle(df["yaw_est"].iloc[k]   - df["yaw_bench"].iloc[k]),
            df["vx_est"].iloc[k] - df["vx_bench"].iloc[k],
            df["vy_est"].iloc[k] - df["vy_bench"].iloc[k],
            df["vz_est"].iloc[k] - df["vz_bench"].iloc[k],
            df["px_est"].iloc[k] - df["px_bench"].iloc[k],
            df["py_est"].iloc[k] - df["py_bench"].iloc[k],
            df["pz_est"].iloc[k] - df["pz_bench"].iloc[k],
        ])

        P = np.diag([
            df["roll_cov_est"].iloc[k],
            df["pitch_cov_est"].iloc[k],
            df["yaw_cov_est"].iloc[k],
            df["vx_cov_est"].iloc[k],
            df["vy_cov_est"].iloc[k],
            df["vz_cov_est"].iloc[k],
            df["px_cov_est"].iloc[k],
            df["py_cov_est"].iloc[k],
            df["pz_cov_est"].iloc[k],
        ])

        nees_k = e.T @ np.linalg.solve(P, e)
        nees.append(nees_k)

        # Debugg
        print(f"e[{k}]: \n {e}")
        print(f"P[{k}]: \n {P}")
        print(f"nees[{k}]: {nees_k}")

    nees = np.array(nees)

    # ---------------- PLOT ----------------
    dof = 9
    confidence = 0.95

    fig, axes = create_stacked_plot(
        1,
        title="NEES Consistency Test (Full State)",
        xlabel="Time [s]",
        ylabels=["NEES"]
    )

    ax = axes[0]

    add_series(
        ax,
        df["t_rel"],
        nees,
        label="NEES",
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
        nees
    )

    lower = chi2.ppf(1 - confidence, dof)
    upper = chi2.ppf(confidence, dof)

    inside = np.logical_and(nees >= lower, nees <= upper)
    percentage_inside = 100.0 * np.sum(inside) / len(nees)

    print(f"NEES inside 95% bounds: {percentage_inside:.2f}%")

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