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

        # Create a figure
        fig, axs = plt.subplots(2)

        # Plot the theta estimates
        axs[0].plot(timestamps, theta_estimates)
        axs[0].set(xlabel="Time (s)", ylabel="Theta Estimate (rad)", title="Theta Estimates")

        # Plot the omega estimates
        axs[1].plot(timestamps, omega_estimates)
        axs[1].set(xlabel="Time (s)", ylabel="Omega Estimate (rad/s)", title="Omega Estimates")

        # Display the plots
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    input_file = Path(__file__).parent.parent / "data_output" / "system_observer_monitor.csv"
    output_file = Path(__file__).parent.parent / "data_output" / "system_observer_analysis.pdf"
    process_data(input_file, output_file)
