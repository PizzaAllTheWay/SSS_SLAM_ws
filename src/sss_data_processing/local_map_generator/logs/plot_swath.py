import os
from utils import (
    get_newest_file,
    load_csv,
    plot_swath,
)



DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

def main():
    raw = load_csv(get_newest_file(DATA_DIR, "swath_raw"))
    proc = load_csv(get_newest_file(DATA_DIR, "swath_processed"))

    plot_swath(raw, proc, y_max=1000)

if __name__ == "__main__":
    main()
