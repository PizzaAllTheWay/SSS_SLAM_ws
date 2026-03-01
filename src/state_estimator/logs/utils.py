import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2



# FILE HELPERS ----------
def get_newest_file(directory, prefix=None):
    pattern = "*.csv" if prefix is None else f"{prefix}_*.csv"
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError("No matching csv files found")
    return max(files, key=os.path.getctime)

def load_csv(path):
    return pd.read_csv(path)

# PLOT HELPER ----------
def finalize_plot():
    for ax in plt.gcf().axes:
        if ax.get_legend_handles_labels()[0]:
            ax.legend()
    plt.tight_layout()
    plt.show()

# 3D PLOT ----------
def create_3d_plot(title="Title",
                   xlabel="X",
                   ylabel="Y",
                   zlabel="Z"):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    return fig, ax

def add_trajectory(ax,
                   df,
                   x_col,
                   y_col,
                   z_col,
                   label=None,
                   color=None,
                   linestyle="-",
                   linewidth=1.5):
    ax.plot(
        df[x_col],
        df[y_col],
        df[z_col],
        label=label,
        color=color,
        linestyle=linestyle,
        linewidth=linewidth
    )

# STACKED GENERIC PLOT ----------
def create_stacked_plot(n_rows,
                        title="Title",
                        xlabel="Time",
                        ylabels=None,
                        sharex=True):
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

def add_series(ax,
               x,
               y,
               label=None,
               color=None,
               linestyle="-",
               linewidth=1.5,
               cov=None,
               sigma=2.0,
               alpha=0.2):
    """
    Generic 2D line plot with optional covariance band.

    cov: variance array (not sigma!)
    sigma: multiplier (default 2σ)
    """

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

# NIS GENERIC PLOT ----------
def add_nis_consistency_bounds(ax,
                               dof,
                               confidence=0.95,
                               label_prefix="Chi²",
                               color="black",
                               linestyle="--"):
    """
    Adds upper/lower chi-square consistency bounds and expected mean.
    """

    upper = chi2.ppf(confidence, dof)
    lower = chi2.ppf(1.0 - confidence, dof)
    expected = dof

    ax.axhline(upper, color=color, linestyle=linestyle,
               label=f"{label_prefix} upper ({confidence*100:.0f}%)")

    ax.axhline(lower, color=color, linestyle=linestyle,
               label=f"{label_prefix} lower ({confidence*100:.0f}%)")

    ax.axhline(expected, color="green", linestyle=linestyle,
               label=f"Expected mean = {dof}")

def add_cumulative_mean(ax,
                        x,
                        y,
                        color="orange",
                        label="Cumulative Mean"):
    """
    True statistical running mean:
    mean_k = (1/k) * sum_{i=1..k} NIS_i
    """
    y = np.asarray(y)
    cumulative_mean = np.cumsum(y) / np.arange(1, len(y) + 1)

    ax.plot(
        x,
        cumulative_mean,
        color=color,
        linewidth=2.5,
        label=label
    )