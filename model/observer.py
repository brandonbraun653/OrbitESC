import numpy as np


class FullStateFeedbackObserver:
    """
    Observer for estimating system output of a motor model. Some assumptions are made about
    the motor's model and is intended to work in conjunction with the model in "motor.py".

    The actual observer is taken from Discrete Time Control Systems by Katsuhiko Ogata, section 6-6, Fig. 6-8.
    """

    def __init__(self, a: np.ndarray, b: np.ndarray, c: np.ndarray):
        """
        Args:
            a: Motor state space 'A' matrix
            b: Motor state space 'B' matrix
            c: Motor state space 'C' matrix
        """
        n = a.shape[0]
        r = b.shape[1]
        m = c.shape[0]

        # Public tunable matrices
        self.K = np.ndarray((r, n), dtype=float)    # State feedback gain matrix
        self.Ke = np.ndarray((n, m), dtype=float)   # Observer feedback gain matrix
        self.G = a
        self.H = b
        self.C = c

        # Output Variables
        self.y_hat = np.ndarray((2, 1), dtype=float)     # Estimated plant output

        # State space variables
        self.x_hat = np.ndarray((4, 1), dtype=float)        # Estimated system state
        self.x_hat_dot = np.ndarray((4, 1), dtype=float)    # Derivative of estimated system state

    def initialize(self) -> None:
        """
        Resets the observer back to initial defaults
        Returns:
            None
        """
        self.K = np.zeros(self.K.shape)
        self.Ke = np.zeros(self.Ke.shape)
        self.y_hat = np.zeros(self.y_hat.shape)
        self.x_hat = np.zeros(self.x_hat.shape)
        self.x_hat_dot = np.zeros(self.x_hat_dot.shape)

        # Assign the observer feedback gain matrix
        self.Ke[0][0] = 1.0
        self.Ke[2][0] = -5.0
        self.Ke[3][1] = -6.0

        assert self._is_stable()

        # Assign the state feedback gain matrix
        self.K[0][0] = 1.0

    def step(self, y_plant: np.ndarray, u: np.ndarray, dt: float) -> None:
        """
        Steps the observer through a single cycle of estimation

        Args:
            y_plant: Measured output of the system being observed
            u: Input control signal to the system being observed
            dt: Time delta in seconds since last step

        Returns:
            None
        """
        # Calculate the output error signal
        output_error_raw = y_plant - self.y_hat

        # Calculate the estimated state
        self.x_hat_dot = self.G @ self.x_hat + self.H @ u + self.Ke @ output_error_raw
        self.x_hat = self.x_hat_dot * dt

        # Calculate the estimated output
        self.y_hat = self.C @ self.x_hat

    def estimated_system_state(self) -> np.ndarray:
        return self.x_hat

    def estimated_system_output(self) -> np.ndarray:
        return self.y_hat

    def system_control_output(self) -> np.ndarray:
        return self.K @ self.x_hat

    def _is_stable(self) -> bool:
        """
        Checks if the gain matrix Ke is stable, which guarantees an error convergence to zero. A matrix is
        considered stable if all eigenvalues are negative.

        Returns:
            True if stable, False if not
        """
        eigenvalues = np.linalg.eigvals(self.G - self.Ke @ self.C)
        check = [np.real(val) < 0.0 for val in eigenvalues]
        return all(check)
