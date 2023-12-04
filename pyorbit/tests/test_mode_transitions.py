from pyorbit.tests.fixtures import *
from pyorbit.serial.client import OrbitClient


@pytest.mark.usefixtures("serial_client")
class TestSystemControlCommands:

    def test_transition_to_idle(self, serial_client: OrbitClient) -> None:
        """ Checks behavior on transition to idle state. """

