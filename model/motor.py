import control
import numpy as np
from loguru import logger


class Motor:
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


if __name__ == "__main__":
    motor = Motor()
    motor.initialize()
    if motor.is_completely_observable:
        logger.info("Motor is controllable")
    else:
        logger.info("Motor isn't controllable")
