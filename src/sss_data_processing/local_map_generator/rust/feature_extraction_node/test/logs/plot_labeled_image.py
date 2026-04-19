import os
from utils import (
    get_newest_file,
    load_csv,
    plot_segmented_and_labeled,
)

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(__file__)
    LOG_DATA_DIR = os.path.join(BASE_DIR, "data")

    segmented_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "segmented_image"),
    )

    landmark_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "landmark_set"),
    )

    plot_segmented_and_labeled(
        segmented_df,
        landmark_df,
        title="Before/After Labelling",
    )
