import numpy as np


class Motor:
    """ BLDC motor model from JamesMevey2009 thesis, equations 6.18 & 6.19 """

    def __init__(self):
        # Public Tunable Constants
        self.R = 0.25     # Resistance
        self.Ls = 0.1     # Stator Inductance
        self.W = 0.0      # Omega -- Rotor speed

        # State space variables
        self._ss_x_dot = np.ndarray((4, 1), dtype=float)
        self._ss_A = np.ndarray((4, 4), dtype=float)
        self._ss_x = np.ndarray((4, 1), dtype=float)
        self._ss_B = np.ndarray((2, 2), dtype=float)

        self._ss_y = np.ndarray((2, 1), dtype=float)
        self._ss_C = np.ndarray((2, 4), dtype=float)

    def initialize(self) -> None:
        """
        Resets the motor model back to initial defaults
        Returns:
            None
        """
        # Initialize the state vector, its derivative, and the output vector
        self._ss_x = np.zeros(self._ss_x.shape)
        self._ss_x_dot = np.zeros(self._ss_x.shape)
        self._ss_y = np.zeros(self._ss_y.shape)

        # Set up the coefficients for the A matrix
        self._ss_A = np.zeros(self._ss_A.shape)
        self._ss_A[0][0] = (-1.0 * self.R)/self.Ls
        self._ss_A[1][1] = (-1.0 * self.R)/self.Ls
        self._ss_A[0][2] = -1.0/self.Ls
        self._ss_A[1][3] = -1.0/self.Ls
        self._ss_A[2][3] = -1.0*self.W
        self._ss_A[3][2] = self.W

        # Set up the coefficients for the B matrix
        self._ss_B = np.zeros(self._ss_B.shape)
        self._ss_B[0][0] = 1.0/self.Ls
        self._ss_B[1][1] = 1.0/self.Ls

        # Set up the coefficients for the C matrix
        self._ss_C = np.zeros(self._ss_C.shape)
        self._ss_C[0][0] = 1.0
        self._ss_C[1][1] = 1.0

    def step(self, omega: float, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Steps the motor model forward by some time delta

        Args:
            omega: Estimated rotor speed from observer
            u: System control input
            dt: Time delta in seconds

        Returns:
            System output
        """
        assert u.shape() == (2, 1)

        # Update the model with the estimated rotor speed
        self.W = omega

        # Step the state space model forward by one iteration
        self._ss_x_dot = self._ss_A * self._ss_x + self._ss_B * u
        self._ss_x = self._ss_x_dot * dt

        # Update the output for this iteration
        self._ss_y = self._ss_C * self._ss_x

        return self._ss_y

    def get_output(self) -> np.ndarray:
        return self._ss_y


if __name__ == "__main__":
    motor = Motor()
    motor.initialize()
