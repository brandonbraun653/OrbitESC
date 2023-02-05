import time
from pyorbit.tests.fixtures import *
from pyorbit.serial_client import SerialClient
from pyorbit.serial_messages import ParameterId


@pytest.mark.usefixtures("serial_client")
class TestSystemControlCommands:

    def test_system_reset(self, serial_client: SerialClient):
        boot_count_prev = serial_client.get_parameter(ParameterId.BootCount)
        time.sleep(1.0)
        result = serial_client.system_reset()
        assert result is True
        time.sleep(2.0)
        boot_count_new = serial_client.get_parameter(ParameterId.BootCount)
        assert (boot_count_new - boot_count_prev) == 1
