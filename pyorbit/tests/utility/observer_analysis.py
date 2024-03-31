import os
import csv
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from pathlib import Path

import matplotlib.pyplot as plt


def process_data(input_file: Path, output_file: Path) -> None:
    """
    Process the current loop data from the input file.
    Args:
        input_file: The input file containing the current loop data.
        output_file: The output file to save the plots.
    """
    with input_file.open() as csvfile:
        csv_data = [row for row in csv.reader(csvfile)]
        timestamps = np.array([float(row[0]) / 1e6 for row in csv_data[1:]])
        theta_estimates = np.array([float(row[1]) for row in csv_data[1:]])
        omega_estimates = np.array([float(row[2]) for row in csv_data[1:]])

    # with PdfPages(output_file) as pdf:
        # Plot the theta estimates
        plt.figure()
        plt.plot(timestamps, theta_estimates)
        plt.xlabel("Time (s)")
        plt.ylabel("Theta Estimate (rad)")
        plt.title("Theta Estimates")
        # pdf.savefig()
        # plt.close()
        plt.show()

        # Plot the omega estimates
        plt.figure()
        plt.plot(timestamps, omega_estimates)
        plt.xlabel("Time (s)")
        plt.ylabel("Omega Estimate (rad/s)")
        plt.title("Omega Estimates")
        # pdf.savefig()
        # plt.close()
        plt.show()


if __name__ == "__main__":
    input_file = Path(__file__).parent.parent / "data_output" / "system_observer_monitor.csv"
    output_file = Path(__file__).parent.parent / "data_output" / "system_observer_analysis.pdf"
    process_data(input_file, output_file)
