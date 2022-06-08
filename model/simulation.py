import matplotlib.pyplot as plt
import numpy as np
from motor import Motor
from observer import FullStateFeedbackObserver


def main() -> None:
    motor = Motor()
    motor.initialize()

    observer = FullStateFeedbackObserver(a=motor.A, b=motor.B, c=motor.C)
    observer.initialize()

    reference = np.zeros((2, 1))
    reference[0, :] = 0.5
    reference[1, :] = 0.8

    sys_ctrl_error = np.zeros((2, 1))
    omega = 1.0

    # Simulate the motor behavior
    time = np.arange(0, 0.050, 0.001)
    data = np.zeros((4, time.shape[0]), dtype=float)

    for idx in range(time.shape[0]):
        u = reference - sys_ctrl_error
        motor.step(omega=omega, u=u, dt=0.001)
        observer.step(y_plant=motor.system_output(), u=u, dt=0.001)
        sys_ctrl_error = observer.system_control_output()
        data[:, idx] = motor.system_state()[:, 0]

        # Calculate the new rotor speed
        # Is this done with equation 6.16 and solving for omega?? Maybe a numerical derivate works too.

    # Plot the output
    fig, ax = plt.subplots()
    ax.plot(time, data[1][:])
    ax.set(xlabel='time (s)', ylabel='state variable', title='Motor State')
    ax.grid()
    plt.show(block=True)


if __name__ == "__main__":
    main()
