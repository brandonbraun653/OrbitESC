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
        phase_a_voltage = np.array([float(row[1]) for row in csv_data[1:]])
        phase_b_voltage = np.array([float(row[2]) for row in csv_data[1:]])
        phase_c_voltage = np.array([float(row[3]) for row in csv_data[1:]])
        alpha_voltage = np.array([float(row[4]) for row in csv_data[1:]])
        beta_voltage = np.array([float(row[5]) for row in csv_data[1:]])

    # with PdfPages(output_file) as pdf:
        # Plot the phase voltages
        plt.figure()
        plt.plot(timestamps, phase_a_voltage)
        plt.plot(timestamps, phase_b_voltage)
        plt.plot(timestamps, phase_c_voltage)
        plt.xlabel("Time (s)")
        plt.ylabel("Phase Voltage (V)")
        plt.title("Phase Voltages")
        plt.legend(["A", "B", "C"])
        # pdf.savefig()
        # plt.close()
        plt.show()

        # Plot the alpha and beta voltages
        plt.figure()
        plt.plot(timestamps, alpha_voltage)
        plt.plot(timestamps, beta_voltage)
        plt.xlabel("Time (s)")
        plt.ylabel("Voltage (V)")
        plt.title("Alpha and Beta Voltages")
        plt.legend(["Alpha", "Beta"])
        # pdf.savefig()
        # plt.close()
        plt.show()


if __name__ == "__main__":
    input_file = Path(__file__).parent.parent / "data_output" / "inner_loop_voltage_monitor.csv"
    output_file = Path(__file__).parent.parent / "data_output" / "inner_loop_voltage_monitor.pdf"
    process_data(input_file, output_file)
