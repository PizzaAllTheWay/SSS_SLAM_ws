import os
import numpy as np

from utils import (
    get_newest_file,
    load_csv,
    build_and_save_partial_map_mosaic_from_time_window,
    plot_saved_partial_mosaic_with_feature_poses_and_landmarks,
)



BASE_DIR = os.path.dirname(__file__)

MAP_DIR = os.path.join(BASE_DIR, "data/map")
FEATURE_DIR = os.path.join(BASE_DIR, "data/feature")
OUT_DIR = os.path.join(BASE_DIR, "data/map/mosaic")

def main():
    # ? NOTE: `YAW_OFFSET` is currently applied as a fixed alignment correction
    # ? between the pose yaw convention and the map/sonar ground-plane convention.
    # ? This works for the current setup, but ideally the underlying frame definition
    # ? should be made fully consistent so this extra offset is no longer needed.
    YAW_OFFSET = (-np.pi/2.0)

    START_T = 1756381500.0
    END_T = 1756381800.0



    map_df = load_csv(get_newest_file(MAP_DIR, "map"))
    map_origin_df = load_csv(get_newest_file(MAP_DIR, "map_origin"))
    map_pose_df = load_csv(get_newest_file(MAP_DIR, "map_pose"))

    feature_pose_df = load_csv(get_newest_file(FEATURE_DIR, "map_pose"))
    landmark_df = load_csv(get_newest_file(FEATURE_DIR, "landmarks"))

    _, meta_path = build_and_save_partial_map_mosaic_from_time_window(
        map_df=map_df,
        map_origin_df=map_origin_df,
        map_pose_df=map_pose_df,
        out_dir=OUT_DIR,
        t_start=START_T,
        t_stop=END_T,
        mosaic_name="mission_partial_features",
        stride=1,
        pixel_stride=1,
    )

    plot_saved_partial_mosaic_with_feature_poses_and_landmarks(
        meta_path=meta_path,
        feature_pose_df=feature_pose_df,
        landmark_df=landmark_df,
        map_origin_df=map_origin_df,
        map_pose_df=map_pose_df,
        t_start=START_T,
        t_stop=END_T,
        title="Map with Trajectory and Landmark Measurements",
        yaw_offset=YAW_OFFSET,
    )


if __name__ == "__main__":
    main()