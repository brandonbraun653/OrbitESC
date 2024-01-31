import logging
from pyorbit.nanopb.motor_control_pb2 import MotorCtrlState
from pyorbit.tests.fixtures import *
from pyorbit.serial.client import OrbitClient

LOGGER = logging.getLogger(__name__)


@pytest.mark.usefixtures("serial_client")
class TestSystemControlCommands:

    def test_transition_to_idle(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to idle state. """
        LOGGER.info("Transitioning to idle state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

    def test_arm_transition(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to arm state. """
        LOGGER.info("Transitioning to idle state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

        LOGGER.info("Validate IDLE->ARM transition")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ARMED)

    def test_arm_idle_transition(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to arm state. """
        LOGGER.info("Transitioning to idle state")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)

        LOGGER.info("Validate IDLE->ARM transition")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_ARMED)

        LOGGER.info("Validate ARM->IDLE transition")
        assert serial_client.set_motor_ctrl_state(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)
