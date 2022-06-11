import matplotlib.pyplot as plt
import numpy as np
from motor import MotorV1, MotorV2
from observer import FullStateFeedbackObserver
from simple_pid import PID


def park_transform(theta, alpha, beta):
    co = np.cos(theta)
    si = np.sin(theta)

    d = co * alpha + si * beta
    q = co * beta - si * alpha

    return d, q


def inv_park_transform(theta, d, q):
    co = np.cos(theta)
    si = np.sin(theta)

    alpha = co * d - si * q
    beta = si * d + co * q

    return alpha, beta


def main() -> None:
    motor = MotorV1()
    motor.initialize()

    observer = FullStateFeedbackObserver(a=motor.A, b=motor.B, c=motor.C)
    observer.initialize()
    observer.Ls = motor.Ls

    reference = np.zeros((2, 1))
    reference[0, :] = -0.5
    reference[1, :] = 0.8

    sys_ctrl_error = np.zeros((2, 1))

    # Simulate the motor behavior
    time = np.arange(0, 0.050, 0.001)
    motor_state_data = np.zeros((4, time.shape[0]), dtype=float)
    rotor_speed_data = np.zeros((1, time.shape[0]), dtype=float)

    d_act = 0
    q_act = 0

    d_pid = PID(1, 0.1, 0.0, setpoint=1.0)
    q_pid = PID(1, 0.1, 0.0, setpoint=0.0)

    for idx in range(time.shape[0]):
        # PI control loop [Id_ref, Iq_ref] => [Vd, Vq]
        d_cmd = d_pid(d_act)
        q_cmd = q_pid(q_act)

        # Push through the inverse park transform [Vd, Vq] => [Va, Vb]
        reference[0, :], reference[1, :] = inv_park_transform(observer.theta, d_cmd, q_cmd)

        # Step through the model
        u = reference - sys_ctrl_error
        motor.step(omega=observer.W, u=u, dt=0.001)
        observer.step(y_plant=motor.system_output(), u=u, dt=0.001)
        sys_ctrl_error = observer.system_control_output()

        # Log some data
        motor_state_data[:, idx] = motor.system_state()[:, 0]
        rotor_speed_data[:, idx] = observer.theta

        # Push through the park transform [Ia, Ib] => [Id, Iq]
        output = motor.system_output()
        d_act, q_act = park_transform(observer.theta, output.item(0), output.item(1))

    # Plot the output
    fig, ax = plt.subplots()
    # ax.plot(time, motor_state_data[1][:])
    ax.plot(time, rotor_speed_data[0][:])
    ax.set(xlabel='time (s)', ylabel='state variable', title='Motor State')
    ax.grid()
    plt.show(block=True)


def main2():
    motor = MotorV2()
    motor.initialize()

    step = 30e-6
    time = np.arange(0, 0.100, step)
    rotor_speed_data = np.zeros((1, time.shape[0]), dtype=float)

    u = np.zeros((2, 1))

    id_act = 0
    iq_act = 0
    id_pid = PID(1, 0.1, 0.0, setpoint=0.0)
    iq_pid = PID(1, 0.1, 0.0, setpoint=0.0)
    rpm_pid = PID(1, 0.1, 0.0, setpoint=0.0)

    rpm_desired = 100

    for idx in range(time.shape[0]):
        # Speed control loop
        rpm_error = rpm_desired - motor.rotor_speed_rpm()
        iq_ref = rpm_pid(rpm_error)
        id_ref = 0

        # PI control loop [Id_ref, Iq_ref] => [Vd, Vq]
        id_cmd = id_pid(id_ref - id_act)
        iq_cmd = iq_pid(iq_ref - iq_act)

        # Calculate vd
        u[0, :] = id_cmd - (iq_act * motor.Ls * motor.hW)

        # Calculate vq
        u[1, :] = iq_cmd + (motor.hW * motor.psi) + (motor.hW * motor.Ls * id_act)

        # Step through the model
        motor.step(u=u, dt=step)

        # Log some data
        rotor_speed_data[:, idx] = motor.rotor_angle_rad()

        # Push through the park transform [Ia, Ib] => [Id, Iq]
        output = motor.system_output()
        id_act, iq_act = park_transform(motor.hTheta, output.item(0), output.item(1))

    # Plot the output
    fig, ax = plt.subplots()
    ax.plot(time, rotor_speed_data[0][:])
    ax.set(xlabel='time (s)', ylabel='state variable', title='Motor State')
    ax.grid()
    plt.show(block=True)


if __name__ == "__main__":
    main2()
