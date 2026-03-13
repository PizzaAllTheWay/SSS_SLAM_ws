from utils import load_all_perf, plot_metric

WINDOW = 0.2

DATASETS = {
    "state_estimator": "src/sss_data_processing/state_estimator/logs/data/performance_20260313_033514.csv",
}

def main():

    data = load_all_perf(DATASETS, WINDOW)

    plot_metric(data, "runtime", "Runtime [ms]", "Callback Runtime", total_color="blue")
    plot_metric(data, "cpu", "CPU [% core]", "CPU Usage", total_color="red")
    plot_metric(data, "ram", "RAM [MB]", "RAM Usage", total_color="green")

if __name__ == "__main__":
    main()
