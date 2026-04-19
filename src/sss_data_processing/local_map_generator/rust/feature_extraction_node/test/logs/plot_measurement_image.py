import os
import numpy as np
from utils import (
    get_newest_file,
    load_csv,
    plot_landmark_measurements,
)

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(__file__)
    LOG_DATA_DIR = os.path.join(BASE_DIR, "data")

    # ? NOTE: `YAW_OFFSET` is currently applied as a fixed alignment correction
    # ? between the pose yaw convention and the map/sonar ground-plane convention.
    # ? This works for the current setup, but ideally the underlying frame definition
    # ? should be made fully consistent so this extra offset is no longer needed.
    YAW_OFFSET = -np.pi/2.0

    filtered_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "filtered_image"),
    )

    landmark_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "landmark_set"),
    )

    plot_landmark_measurements(
        filtered_df,
        landmark_df,
        title="Landmark Measurements",
        yaw_offset = YAW_OFFSET,
    )
