import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def process_current_loop_data(input_file: Path) -> None:
    """
    Process the current loop data from the input file.
    Args:
        input_file: The input file containing the current loop data.
    """
    with input_file.open() as csvfile:
        csv_data = [row for row in csv.reader(csvfile)]
        timestamps = np.array([float(row[0]) / 1e6 for row in csv_data[1:]])
        phase_currents = np.array([[float(row[i]) for i in range(1, 4)] for row in csv_data[1:]])
        dq_reference_currents = np.array([[float(row[i]) for i in range(4, 6)] for row in csv_data[1:]])
        dq_measured_currents = np.array([[float(row[i]) for i in range(6, 8)] for row in csv_data[1:]])
        dq_voltage_commands = np.array([[float(row[i]) for i in range(8, 10)] for row in csv_data[1:]])
        ab_voltage_commands = np.array([[float(row[i]) for i in range(10, 12)]for row in csv_data[1:]])

    # # Plot the phase currents
    # plt.figure()
    # plt.plot(timestamps, phase_currents)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Phase Current (A)")
    # plt.title("Phase Currents")
    # plt.legend(["A", "B", "C"])
    # plt.show()

    # # Plot the DQ measured currents
    # plt.figure()
    # plt.plot(timestamps, dq_measured_currents)
    # plt.xlabel("Time (s)")
    # plt.ylabel("DQ Current (A)")
    # plt.title("DQ Measured Currents")
    # plt.legend(["D", "Q"])
    # plt.show()

    # # Plot dq voltage commands vs dq measured currents
    # fig, axs = plt.subplots(2)
    #
    # # Subplot for DQ voltage commands
    # axs[0].plot(timestamps, dq_voltage_commands)
    # axs[0].set(xlabel="Time (s)", ylabel="Voltage (V)", title="DQ Voltage Commands")
    # axs[0].legend(["D", "Q"])
    #
    # # Subplot for DQ measured currents
    # axs[1].plot(timestamps, dq_measured_currents)
    # axs[1].set(xlabel="Time (s)", ylabel="DQ Current (A)", title="DQ Measured Currents")
    # axs[1].legend(["D", "Q"])
    #
    # # Display the figure
    # plt.tight_layout()
    # plt.show()

    # # Plot IqRef vs Iq and IdRef vs Id
    # fig, axs = plt.subplots(2)
    #
    # # Subplot for IqRef vs Iq
    # axs[0].plot(timestamps, dq_reference_currents[:, 1])
    # axs[0].plot(timestamps, dq_measured_currents[:, 1])
    # axs[0].set(xlabel="Time (s)", ylabel="Current (A)", title="IqRef vs Iq")
    #
    # # Subplot for IdRef vs Id
    # axs[1].plot(timestamps, dq_reference_currents[:, 0])
    # axs[1].plot(timestamps, dq_measured_currents[:, 0])
    # axs[1].set(xlabel="Time (s)", ylabel="Current (A)", title="IdRef vs Id")
    #
    # # Display the figure
    # plt.tight_layout()
    # plt.show()

    # # Plot Iq Error vs Vq Command
    # err = 0.01 * (dq_reference_currents[:, 1] - dq_measured_currents[:, 1])
    # plt.figure()
    # plt.plot(timestamps, err)
    # plt.plot(timestamps, dq_voltage_commands[:, 1])
    # plt.xlabel("Time (s)")
    # plt.ylabel("Value")
    # plt.title("Iq Error vs Vq Command")
    # plt.legend(["Iq Error", "Vq Command"])
    # plt.show()

    # Plot Alpha-Beta voltage commands
    plt.figure()
    plt.plot(timestamps, ab_voltage_commands)
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.title("Alpha-Beta Voltage Commands")
    plt.legend(["Alpha", "Beta"])
    plt.show()


if __name__ == "__main__":
    data_file = Path(__file__).parent.parent / "data_output" / "current_control_monitor.csv"
    process_current_loop_data(data_file)
