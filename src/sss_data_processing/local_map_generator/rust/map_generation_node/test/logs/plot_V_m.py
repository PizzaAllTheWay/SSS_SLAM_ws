import os
import numpy as np
from utils import (
    get_newest_file,
    load_csv,
    plot_vm_and_swath_interactive,
)



if __name__ == "__main__":
    DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

    # ! Debugging
    # START_T = None
    # END_T = None
    # 5000 samples
    START_T = 1756381550.0
    END_T = 1756381750.0
    # Test samples
    # START_T = 1756381086.0
    # END_T = 1756381250.0

    # ! YAW OFFSET
    YAW_OFFSET = -np.pi/2.0

    cell_map_m_df = load_csv(
        get_newest_file(DATA_DIR, "cell_map_m"),
        start_t=START_T,
        end_t=END_T,
    )

    pose_df = load_csv(
        os.path.join(os.path.dirname(__file__), "../data/pose_interpolated.csv"),
        start_t=START_T,
        end_t=END_T,
    )

    swath_df = load_csv(
        os.path.join(os.path.dirname(__file__), "../data/swath_processed.csv"),
        start_t=START_T,
        end_t=END_T,
    )

    plot_vm_and_swath_interactive(
        cell_map_m_df,
        pose_df,
        swath_df,
        title="V_m and Processed Swath",
        yaw_offset=YAW_OFFSET,
    )

