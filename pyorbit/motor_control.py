import pyorbit.nanopb.serial_interface_pb2 as proto
from enum import Enum
from loguru import logger
from typing import Callable, List
from pyorbit.serial.client import OrbitClient
from pyorbit.serial.messages import AckNackMessage, MotorControlMessage


class MotorControl:
    """ High level motor controller class intended for testing purposes """

    class ActiveState(Enum):
        """ Active state of the motor controller """
        Disabled = proto.DISABLE_OUTPUT_STAGE
        Enabled = proto.ENABLE_OUTPUT_STAGE

    def __init__(self, serial_client: OrbitClient):
        self._serial = serial_client

    def close(self):
        self._serial.close()

    def set_power_stage(self, state: ActiveState) -> bool:
        """
        Control the liveness of the power stage
        Args:
            state: What state to transition the power stage to

        Returns:
            True if the command was successful, False otherwise
        """
        return self._command_transaction(cmd=state.value)

    def emergency_stop(self) -> bool:
        """
        Emergency stop the motor controller
        Returns:
            True if the command was successful, False otherwise
        """
        return self._command_transaction(cmd=MotorControlMessage.Command.EmergencyStop)

    def _command_transaction(self, cmd: MotorControlMessage.Command, data: bytes = None,
                             validator: Callable[[List[AckNackMessage]], bool] = None) -> bool:
        """
        Send a command to the motor controller and wait for a response

        Args:
            cmd: Command to send, from protobuf MotorCtrlCmd
            data: Data to pack with the command message
            validator: Function to validate the response

        Returns:
            True if the command was successful, False otherwise
        """
        sub_id = self._serial.com_pipe.subscribe(msg=AckNackMessage, qty=1, timeout=5.0)
        self._serial.com_pipe.write(MotorControlMessage(cmd, data).serialize())
        responses = self._serial.com_pipe.get_subscription_data(sub_id, terminate=True)  # type: List[AckNackMessage]

        if not responses:
            logger.warning("No response for motor control command")
            return False

        if validator:
            return validator(responses)
        else:
            return responses[0].ack
