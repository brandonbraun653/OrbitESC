from pyorbit.nanopb.motor_control_pb2 import MotorCtrlState
from pyorbit.tests.fixtures import *
from pyorbit.serial.client import OrbitClient


@pytest.mark.usefixtures("serial_client")
class TestSystemControlCommands:

    def test_transition_to_idle(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to idle state. """
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

    def test_arm_idle_transition(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to arm state. """
        # Start off in idle state
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

        # Attempt to ARM
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ARMED)

        # Go back to idle to ensure the other direction works
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)
