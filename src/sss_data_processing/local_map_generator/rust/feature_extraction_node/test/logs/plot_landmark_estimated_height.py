import os
from utils import (
    get_newest_file,
    load_csv,
    plot_landmark_height,
)

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(__file__)
    LOG_DATA_DIR = os.path.join(BASE_DIR, "data")

    filtered_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "filtered_image"),
    )

    landmark_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "landmark_set"),
    )

    plot_landmark_height(
        filtered_df,
        landmark_df,
        title="Map",
    )
