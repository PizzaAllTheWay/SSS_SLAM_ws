import os
from utils import (
    get_newest_file,
    load_csv,
    build_and_save_full_map_mosaic,
    plot_saved_full_map_mosaic,
)

DATA_DIR = os.path.join(os.path.dirname(__file__), "data/map")
OUT_DIR = os.path.join(os.path.dirname(__file__), "data/map/mosaic")

def main():
    map_df = load_csv(get_newest_file(DATA_DIR, "map"))
    map_origin_df = load_csv(get_newest_file(DATA_DIR, "map_origin"))
    map_pose_df = load_csv(get_newest_file(DATA_DIR, "map_pose"))
    
    _, meta_path = build_and_save_full_map_mosaic(
        map_df,
        map_origin_df,
        map_pose_df,
        out_dir=OUT_DIR,
        mosaic_name="mission_full",
        stride=1,        # use every Nth map
        pixel_stride=1,  # keep every Nnd pixel
    )

    meta_path = os.path.join(OUT_DIR, f"mission_full.json")

    plot_saved_full_map_mosaic(meta_path)

if __name__ == "__main__":
    main()