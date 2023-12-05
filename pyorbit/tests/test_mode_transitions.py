from pyorbit.nanopb.motor_control_pb2 import MotorCtrlState
from pyorbit.serial.messages import SystemStatusPBMsg
from pyorbit.tests.fixtures import *
from pyorbit.serial.client import OrbitClient


@pytest.mark.usefixtures("serial_client")
class TestSystemControlCommands:

    def test_transition_to_idle(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to idle state. """
        serial_client.request_motor_state_transition(MotorCtrlState.MOTOR_CTRL_STATE_IDLE)
        packets = serial_client.com_pipe.filter(
            lambda msg: isinstance(msg, SystemStatusPBMsg) and
                        (msg.motor_ctrl_state == MotorCtrlState.MOTOR_CTRL_STATE_IDLE),
            qty=1,
            timeout=5.0)

        assert len(packets) == 1
