from utils import load_all_schedulability, plot_schedulability



T_START = 1756381030.0
T_STOP  = 1756382638.0



DATASETS = {
    "state_estimator": "src/sss_data_processing/state_estimator/logs/data/performance_20260313_033514.csv",
    "swath_processing": "src/sss_data_processing/local_map_generator/logs/data/swath/performance_1774645054.csv",
    "map_generation": "src/sss_data_processing/local_map_generator/logs/data/map/performance_1777148008.csv",
    "feature_extraction": "src/sss_data_processing/local_map_generator/logs/data/feature/performance_1777170409.csv",
}



def main():
    sched = load_all_schedulability(
        DATASETS,
        T_START,
        T_STOP,
        median_samples=2,
        split_datasets={
            "map_generation": {
                "factor": 2.0,
                "low_name": "map_generation",
                "high_name": "map_extraction",
            }
        },
    )

    plot_schedulability(sched, title="Schedulability", smooth_window=100)

if __name__ == "__main__":
    main()