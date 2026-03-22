import os
from utils import (
    get_newest_file,
    load_csv,
    plot_swath,
)



if __name__ == "__main__":
    DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

    START_SAMPLE = 3500
    END_SAMPLE = 4500

    raw_df = load_csv(
        get_newest_file(DATA_DIR, "swath_raw"),
        start_sample=START_SAMPLE,
        end_sample=END_SAMPLE,
    )
    blind_df = load_csv(
        get_newest_file(DATA_DIR, "swath_processed_blindzone"),
        start_sample=START_SAMPLE,
        end_sample=END_SAMPLE,
    )
    norm_df = load_csv(
        get_newest_file(DATA_DIR, "swath_processed_normalized"),
        start_sample=START_SAMPLE,
        end_sample=END_SAMPLE,
    )
    corr_df = load_csv(
        get_newest_file(DATA_DIR, "swath_processed_corrected"),
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
        blind_df,
        title=f"Blind Zone Removed Swath [{START_SAMPLE}:{END_SAMPLE}]",
        ylabel="Ping #",
        xlabel="Across track",
        zero_color="white",
    )

    plot_swath(
        norm_df,
        title=f"Intensity Normalized Swath [{START_SAMPLE}:{END_SAMPLE}]",
        ylabel="Ping #",
        xlabel="Across track",
        zero_color="white",
    )

    plot_swath(
        corr_df,
        title=f"Slant Corrected Swath [{START_SAMPLE}:{END_SAMPLE}]",
        ylabel="Ping #",
        xlabel="Across track",
        zero_color="white",
    )
