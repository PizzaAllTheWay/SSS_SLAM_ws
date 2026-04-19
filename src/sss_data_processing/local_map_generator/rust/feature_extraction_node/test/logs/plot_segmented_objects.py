import os
from utils import (
    get_newest_file,
    load_csv,
    plot_before_after_image,
)

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(__file__)
    LOG_DATA_DIR = os.path.join(BASE_DIR, "data")

    filtered_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "filtered_image"),
    )

    segmented_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "segmented_image"),
    )

    plot_before_after_image(
        before_df=filtered_df,
        after_df=segmented_df,
        title="Before/After Segmentation",
    )