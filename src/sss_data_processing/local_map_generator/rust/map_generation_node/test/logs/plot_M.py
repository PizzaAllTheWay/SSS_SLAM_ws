import os
import numpy as np
from utils import (
    get_newest_file,
    load_csv,
    plot_map_interactive,
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

    map_df = load_csv(
        get_newest_file(DATA_DIR, "map"),
        start_t=START_T,
        end_t=END_T,
    )

    plot_map_interactive(
        map_df,
        title="Side Scan Sonar Map M",
        yaw_offset = YAW_OFFSET,
    )