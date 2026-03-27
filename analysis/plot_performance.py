from utils import load_all_perf, plot_metric



WINDOW = 0.2
T_START = 1756381030.0
T_STOP  = 1756382638.0
T_DELTA = 5.0

DATASETS = {
    "state_estimator": "src/sss_data_processing/state_estimator/logs/data/performance_20260313_033514.csv",
    "swath_processing": "src/sss_data_processing/local_map_generator/logs/data/swath/performance_1774645054.csv",
}



def main():
    data = load_all_perf(DATASETS, WINDOW, T_START, T_STOP, T_DELTA)

    plot_metric(data, "runtime", "Runtime [ms]", "Callback Runtime", total_color="blue")
    plot_metric(data, "cpu", "CPU [% core]", "CPU Usage", total_color="red")
    plot_metric(data, "ram", "RAM [MB]", "RAM Usage", total_color="green")

if __name__ == "__main__":
    main()
