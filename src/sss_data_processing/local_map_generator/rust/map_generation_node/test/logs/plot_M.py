import os
import numpy as np
from utils import (
    get_newest_file,
    load_csv,
    plot_map_interactive,
)

if __name__ == "__main__":
    DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

    # START_T = None
    # END_T = None
    # 5000 samples
    START_T = 1756381550.0
    END_T = 1756381750.0
    TIME_OFFSET = 70.0 # Purely for syncing relative data time with other graphs because teh whole map gets generated later

    # ? NOTE: `YAW_OFFSET` is currently applied as a fixed alignment correction
    # ? between the pose yaw convention and the map/sonar ground-plane convention.
    # ? This works for the current setup, but ideally the underlying frame definition
    # ? should be made fully consistent so this extra offset is no longer needed.
    YAW_OFFSET = -np.pi/2.0

    map_df = load_csv(
        get_newest_file(DATA_DIR, "map"),
        start_t=START_T,
        end_t=END_T,
    )

    plot_map_interactive(
        map_df,
        title="Side Scan Sonar Map M",
        yaw_offset=YAW_OFFSET,
        time_offset=TIME_OFFSET,
    )