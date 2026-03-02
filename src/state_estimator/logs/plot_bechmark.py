import os
from utils import (
    get_newest_file,
    load_csv,
    create_3d_plot,
    create_stacked_plot,
    add_trajectory,
    add_series,
    finalize_plot
)

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

def main():
    bench_file = get_newest_file(DATA_DIR, "benchmark")
    est_file   = get_newest_file(DATA_DIR, "state_estimate")

    df_bench = load_csv(bench_file)
    df_est   = load_csv(est_file)

    # Convert to relative time
    t0_bench = df_bench["t"].iloc[0]
    t0_est   = df_est["t"].iloc[0]

    df_bench["t_rel"] = df_bench["t"] - t0_bench
    df_est["t_rel"]   = df_est["t"]   - t0_est

    # Set relative xyz pose
    x0_bench = df_bench["px"].iloc[0]
    y0_bench = df_bench["py"].iloc[0]
    z0_bench = df_bench["pz"].iloc[0]

    df_bench["px"] = df_bench["px"] - x0_bench
    df_bench["py"] = df_bench["py"] - y0_bench
    df_bench["pz"] = df_bench["pz"] - z0_bench

    # =========================================================
    # 1) 3D TRAJECTORY
    # =========================================================
    fig1, ax1 = create_3d_plot(
        title="3D Pose Comparison",
        xlabel="X [m]",
        ylabel="Y [m]",
        zlabel="Z [m]"
    )

    add_trajectory(ax1, df_bench, "px", "py", "pz",
                   label="Benchmark", color="blue")

    add_trajectory(ax1, df_est, "px", "py", "pz",
                   label="UKF-M", color="red")

    finalize_plot()

    # =========================================================
    # 2) POSITION
    # =========================================================
    fig2, axes_pos = create_stacked_plot(
        3,
        title="Position Comparison",
        xlabel="Time [s]",
        ylabels=["px [m]", "py [m]", "pz [m]"]
    )

    for i, axis in enumerate(["px", "py", "pz"]):
        add_series(
            axes_pos[i],
            df_bench["t_rel"], df_bench[axis],
            label="Benchmark",
            color="blue",
            cov=df_bench[f"{axis}_cov"]
        )

        add_series(
            axes_pos[i],
            df_est["t_rel"], df_est[axis],
            label="UKF-M",
            color="red",
            cov=df_est[f"{axis}_cov"]
        )

    finalize_plot()

    # =========================================================
    # 3) VELOCITY
    # =========================================================
    fig3, axes_vel = create_stacked_plot(
        3,
        title="Velocity Comparison",
        xlabel="Time [s]",
        ylabels=["vx [m/s]", "vy [m/s]", "vz [m/s]"]
    )

    for i, axis in enumerate(["vx", "vy", "vz"]):
        add_series(
            axes_vel[i],
            df_bench["t_rel"], df_bench[axis],
            label="Benchmark",
            color="blue",
            cov=df_bench[f"{axis}_cov"]
        )

        add_series(
            axes_vel[i],
            df_est["t_rel"], df_est[axis],
            label="UKF-M",
            color="red",
            cov=df_est[f"{axis}_cov"]
        )

    finalize_plot()

    # =========================================================
    # 4) ORIENTATION
    # =========================================================
    fig4, axes_ori = create_stacked_plot(
        3,
        title="Orientation Comparison",
        xlabel="Time [s]",
        ylabels=["roll [rad]", "pitch [rad]", "yaw [rad]"]
    )

    for i, axis in enumerate(["roll", "pitch", "yaw"]):
        add_series(
            axes_ori[i],
            df_bench["t_rel"], df_bench[axis],
            label="Benchmark",
            color="blue",
            cov=df_bench[f"{axis}_cov"]
        )

        add_series(
            axes_ori[i],
            df_est["t_rel"], df_est[axis],
            label="UKF-M",
            color="red",
            cov=df_est[f"{axis}_cov"]
        )

    finalize_plot()


if __name__ == "__main__":
    main()