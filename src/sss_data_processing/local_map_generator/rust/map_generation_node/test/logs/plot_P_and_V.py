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

    # START_T = None
    # END_T = None
    # 5000 samples
    START_T = 1756381550.0
    END_T = 1756381750.0

    # ? NOTE: Change these according to your test.rs parameter values
    CHUNK_SIZE = 64
    MAP_RESOLUTION = 30.0/1000.0

    # ? NOTE: `YAW_OFFSET` is currently applied as a fixed alignment correction
    # ? between the pose yaw convention and the map/sonar ground-plane convention.
    # ? This works for the current setup, but ideally the underlying frame definition
    # ? should be made fully consistent so this extra offset is no longer needed.
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