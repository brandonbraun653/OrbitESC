import control
import numpy as np
from scipy.integrate import quad
from loguru import logger


class MotorV1:
    """ BLDC state space motor model from JamesMevey2009 thesis, equations 6.18 & 6.19 """

    def __init__(self):
        # Public Tunable Constants
        self.R = 0.25  # Resistance
        self.Ls = 0.1  # Stator Inductance
        self.W = 0.1  # Omega -- Rotor speed

        # State space variables
        self.xdot = np.ndarray((4, 1), dtype=float)
        self.A = np.ndarray((4, 4), dtype=float)
        self.x = np.ndarray((4, 1), dtype=float)
        self.B = np.ndarray((4, 2), dtype=float)

        self.y = np.ndarray((2, 1), dtype=float)
        self.C = np.ndarray((2, 4), dtype=float)
        self.D = np.ndarray((2, 2), dtype=float)

    def initialize(self) -> None:
        """
        Resets the motor model back to initial defaults
        Returns:
            None
        """
        # Initialize the state vector, its derivative, and the output vector
        self.x = np.zeros(self.x.shape)
        self.xdot = np.zeros(self.x.shape)
        self.y = np.zeros(self.y.shape)

        # Set up the coefficients for the A matrix
        self.A = np.zeros(self.A.shape)
        self.A[0][0] = (-1.0 * self.R) / self.Ls
        self.A[1][1] = (-1.0 * self.R) / self.Ls
        self.A[0][2] = -1.0 / self.Ls
        self.A[1][3] = -1.0 / self.Ls
        self.A[2][3] = -1.0 * self.W
        self.A[3][2] = self.W

        # Set up the coefficients for the B matrix
        self.B = np.zeros(self.B.shape)
        self.B[0][0] = 1.0 / self.Ls
        self.B[1][1] = 1.0 / self.Ls

        # Set up the coefficients for the C matrix
        self.C = np.zeros(self.C.shape)
        self.C[0][0] = 1.0
        self.C[1][1] = 1.0

        # Set up the coefficients for the D matrix
        self.D = np.zeros(self.D.shape)

        # Verify that the system is controllable and observable. These are prerequisites for the control system.
        assert self.is_output_controllable
        assert self.is_completely_observable

    def step(self, omega: float, u: np.ndarray, dt: float) -> None:
        """
        Steps the motor model forward by some time delta

        Args:
            omega: Estimated rotor speed from observer
            u: System control input
            dt: Time delta in seconds since last step

        Returns:
            None
        """
        assert u.shape == (2, 1)

        # Update the model with the estimated rotor speed
        self.W = omega
        self.A[2][3] = -1.0 * self.W
        self.A[3][2] = self.W

        # Step the state space model forward by one iteration
        self.xdot = self.A @ self.x + self.B @ u
        self.x = self.xdot * dt

        # Update the output for this iteration
        self.y = self.C @ self.x

    def system_output(self) -> np.ndarray:
        return self.y

    def system_state(self) -> np.ndarray:
        return self.x

    @property
    def is_output_controllable(self) -> bool:
        """
        Checks if the state space model exhibits the Output Controllability property. This is
        necessary for the motor output to be controlled by some external control system. Full
        state controllability is not required for this model.

        System is Output Controllable if the rank of the matrix below is == m:
            rank([D|CB|CAB|CA^2B|---|CA^(n-1)B]) == m

        Returns:
            True if the system is Output Controllable, else False
        """
        n = self.A.shape[0]
        m = self.C.shape[0]

        out_ctrl_mtx = np.hstack((self.D, np.matmul(self.C, self.B)))
        for i in range(1, n):
            a_pow = np.linalg.matrix_power(self.A, i)
            ca = np.matmul(self.C, a_pow)
            total = np.matmul(ca, self.B)
            out_ctrl_mtx = np.hstack((out_ctrl_mtx, total))

        rank = np.linalg.matrix_rank(out_ctrl_mtx)
        return rank == m

    @property
    def is_completely_observable(self) -> bool:
        """
        Checks if the state space model is completely observable. This is a necessary condition for
        an observer to have a chance at accurately estimating system state parameters.

        Returns:
            True if the system is observable, False if not
        """
        n = self.A.shape[0]
        o = control.obsv(self.A, self.C)
        rank = np.linalg.matrix_rank(o)
        return rank == n


class MotorV2:

    def __init__(self):
        # Constants
        self.Rs = 18.0e-3  # Stator resistance
        self.Ls = 12.8e-6  # Stator inductance
        self.W_max = 2100  # ~20k RPM
        self.num_inner_loop_samples = 5

        # Estimations
        self.hW = 0.0       # Estimated rotor speed in radians/sec
        self.hTheta = 0.0   # Estimated rotor angle in radians

        # State space variables
        self.xdot = np.ndarray((4, 1), dtype=float)
        self.A = np.ndarray((4, 4), dtype=float)
        self.x = np.ndarray((4, 1), dtype=float)
        self.B = np.ndarray((4, 2), dtype=float)
        self.y = np.ndarray((2, 1), dtype=float)
        self.C = np.ndarray((2, 4), dtype=float)

        # Observer Variables
        self.zdot = np.ndarray((2, 1), dtype=float)
        self.z = np.ndarray((2, 1), dtype=float)
        self.hx = np.ndarray((2, 1), dtype=float)
        self.D = np.ndarray((2, 2), dtype=float)
        self.L = np.ndarray((2, 2), dtype=float)
        self.F = np.ndarray((2, 2), dtype=float)
        self.G = np.ndarray((2, 2), dtype=float)

        # General Variables
        self._inner_sample_num = 0
        self._h_theta_last = 0.0

    @property
    def d(self) -> float:
        return (-self.Rs / self.Ls) * 5.0  # (5.0 + (15.0 / self.W_max) * abs(self.hW))

    def initialize(self) -> None:
        # Initialize the state vector, its derivative, and the output vector
        self.x = np.zeros(self.x.shape)
        self.xdot = np.zeros(self.x.shape)
        self.y = np.zeros(self.y.shape)

        # Set up the coefficients for the A matrix
        self.A = np.zeros(self.A.shape)
        self.A[0][0] = -self.Rs / self.Ls
        self.A[0][2] = -1.0 / self.Ls

        self.A[1][1] = -self.Rs / self.Ls
        self.A[1][3] = -1.0 / self.Ls

        self.A[2][3] = -self.hW
        self.A[3][2] = self.hW

        # Set up the coefficients for the B matrix
        self.B = np.zeros(self.B.shape)
        self.B[0][0] = 1.0 / self.Ls
        self.B[1][1] = 1.0 / self.Ls

        # Set up the coefficients for the C matrix
        self.C = np.zeros(self.C.shape)
        self.C[0][0] = 1.0
        self.C[1][1] = 1.0

        # Initialize the observer
        self.zdot = np.zeros(self.zdot.shape)
        self.z = np.zeros(self.z.shape)
        self.hx = np.zeros(self.hx.shape)
        self._update_observer_matrices()

    def _cmn_matrix(self) -> np.ndarray:
        tmp = np.ndarray((2, 2), dtype=float)
        tmp[0][0] = self.d
        tmp[0][1] = -self.hW
        tmp[1][0] = self.hW
        tmp[1][1] = self.d

        return tmp

    def _update_observer_matrices(self) -> None:
        tmp = self._cmn_matrix()

        self.D = self.d * np.eye(2)
        self.L = self.Ls * tmp
        self.F = (self.Rs + self.Ls) * tmp
        self.G = -1.0 * tmp

    def step(self, u: np.ndarray, dt: float) -> None:
        """
        Steps the motor model forward by some time delta

        Args:
            omega: Estimated rotor speed from observer
            u: System control input
            dt: Time delta in seconds since last step

        Returns:
            None
        """
        # Update the model with the estimated rotor speed
        self.A[2][3] = -self.hW
        self.A[3][2] = self.hW

        # Step the state space model forward by one iteration
        self.xdot = self.A @ self.x + self.B @ u
        self.x = self.xdot * dt
        self.y = self.C @ self.x

        # Step the observer forward by one iteration
        Dz = self.D @ self.z
        Fy = (self.Rs + self.d * self.Ls) * self._cmn_matrix() @ self.y
        Gu = self._cmn_matrix() @ u
        self.zdot = Dz + Fy - Gu
        self.z = self.zdot * dt

        # Step the observer output
        self.hx = self.z + self.L @ self.y

        # Check if it's time to do outer loop calculations
        self._inner_sample_num += 1
        if self._inner_sample_num >= self.num_inner_loop_samples:
            T_est = self.num_inner_loop_samples * dt
            self._inner_sample_num = 0

            # Calculate the rotor angle: arctan2(Ed, Eq)
            self.hTheta = np.arctan2(self.hx.item(1), self.hx.item(0))
            if self.hW < 0.0:
                self.hTheta += np.pi

            # Calculate the rotor speed
            self.hW = (self.hTheta - self._h_theta_last) / T_est
            self._h_theta_last = self.hTheta

    def system_output(self) -> np.ndarray:
        return self.y

    def system_state(self) -> np.ndarray:
        return self.x

    def rotor_angle(self) -> float:
        return self.hTheta

    def rotor_speed(self) -> float:
        return self.hW


if __name__ == "__main__":
    motor = MotorV1()
    motor.initialize()
    if motor.is_completely_observable:
        logger.info("Motor is controllable")
    else:
        logger.info("Motor isn't controllable")
