import argparse

import flightgear
import plot_csv
import simple_rot_vis


def run_post_proc(data_log_path):
    """
    Manages porst processing routine from command line arguments.
    """

    if args.plot_csv:
        print("Running plot_csv")
        plot_csv.plot_csv(data_log_path)
    if args.simple_rot_vis:
        print("Running show_simple_rot")
        simple_rot_vis.show_rot(data_log_path)
    if args.flightgear_vis:
        print("Running flightgear_vis")
        flightgear.run(data_log_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot CSV timeseries with paging (3x3).")
    parser.add_argument("--data_log", type=str, required=True, help="Path to CSV data log file")
    parser.add_argument("--plot_csv", action="store_true", required=False, help="Run script to plot CSV log")
    parser.add_argument("--simple_rot_vis", action="store_true", required=False, help="Run script for simple reference frame visualization")
    parser.add_argument("--flightgear_vis", action="store_true", required=False, help="Run script for FlighGear visualization")
    args = parser.parse_args()

    data_log_path = args.data_log

    run_post_proc(data_log_path)