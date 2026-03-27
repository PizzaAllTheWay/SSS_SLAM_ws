import os
from utils import (
    get_newest_file,
    load_csv,
    plot_geometric_correction,
    load_local_map_generator_params,
)

DATA_DIR = os.path.join(os.path.dirname(__file__), "data/swath")
CONFIG_DIR = os.path.join(os.path.dirname(__file__), "../config")
CONFIG_PATH = os.path.join(CONFIG_DIR, "local_map_generator_params.yaml")

def main():
    altitude_dvl = load_csv(get_newest_file(DATA_DIR, "altitude_dvl"))
    geometric_correction = load_csv(get_newest_file(DATA_DIR, "geometric_correction"))
    swath_raw = load_csv(get_newest_file(DATA_DIR, "swath_raw"))

    cfg = load_local_map_generator_params(CONFIG_PATH)

    t_start = 1756381027.016462
    t_stop  = 1756382637.907318

    plot_geometric_correction(
        altitude_df=altitude_dvl,
        geometric_df=geometric_correction,
        raw_df=swath_raw,
        t_start=t_start,
        t_stop=t_stop,
        max_range=cfg["max_range"],
        port_scale=cfg["port_scale"],
        stb_scale=cfg["stb_scale"],
        title="Swath Geometric Correction",
        gap_threshold=5.0,
    )

if __name__ == "__main__":
    main()
