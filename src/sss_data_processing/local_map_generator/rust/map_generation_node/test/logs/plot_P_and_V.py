# plot_chunk_map.py
import os
import numpy as np
from utils import (
    get_newest_file,
    load_csv,
    plot_chunk_map_pv_interactive,
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

    # ! TODO: Should be a import from YAML file !
    CHUNK_SIZE = 64
    MAP_RESOLUTION = 30.0/1000.0

    # ! YAW OFFSET
    YAW_OFFSET = -np.pi/2.0

    chunk_map_df = load_csv(
        get_newest_file(DATA_DIR, "chunk_map"),
        start_t=START_T,
        end_t=END_T,
    )

    pose_df = load_csv(
        os.path.join(os.path.dirname(__file__), "../data/pose_interpolated.csv"),
        start_t=START_T,
        end_t=END_T,
    )

    plot_chunk_map_pv_interactive(
        chunk_map_df,
        pose_df,
        chunk_size=CHUNK_SIZE,
        map_resolution=MAP_RESOLUTION,
        title="Chunk Map: V and P",
        yaw_offset=YAW_OFFSET,
    )