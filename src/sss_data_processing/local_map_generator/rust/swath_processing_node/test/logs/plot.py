import os
from utils import (
    get_newest_file,
    load_csv,
    plot_swath,
)



if __name__ == "__main__":
    DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

    # START_SAMPLE = 3500
    # END_SAMPLE = 4500

    START_SAMPLE = 0
    END_SAMPLE = 9000

    raw_df = load_csv(
        os.path.join(os.path.dirname(__file__), "../data/swath_raw.csv"),
        start_sample=START_SAMPLE,
        end_sample=END_SAMPLE,
    )
    processed_df = load_csv(
        get_newest_file(DATA_DIR, "swath_processed"),
        start_sample=START_SAMPLE,
        end_sample=END_SAMPLE,
    )

    plot_swath(
        raw_df,
        title=f"Raw Swath [{START_SAMPLE}:{END_SAMPLE}]",
        ylabel="Ping #",
        xlabel="Across track",
        zero_color="white",
    )

    plot_swath(
        processed_df,
        title=f"Processed Swath [{START_SAMPLE}:{END_SAMPLE}]",
        ylabel="Ping #",
        xlabel="Across track",
        zero_color="white",
    )