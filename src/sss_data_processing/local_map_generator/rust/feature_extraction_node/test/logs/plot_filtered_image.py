import os
from utils import (
    get_newest_file,
    load_csv,
    plot_before_after_image,
)

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(__file__)
    RAW_DATA_DIR = os.path.join(BASE_DIR, "../data")
    LOG_DATA_DIR = os.path.join(BASE_DIR, "data")

    start_n = 38;
    n = 9;

    raw_df = load_csv(
        os.path.join(RAW_DATA_DIR, "map.csv"),
        start_sample=start_n-1,
        end_sample=start_n+n-1,
    )

    filtered_df = load_csv(
        get_newest_file(LOG_DATA_DIR, "filtered_image"),
    )

    plot_before_after_image(
        before_df=raw_df,
        after_df=filtered_df,
        title="Before/After Filtering",
    )