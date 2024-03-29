import math
import time
import numpy as np
from pyorbit.tests.fixtures import *
from pyorbit.serial.client import OrbitClient
from pyorbit.serial.parameters import ParameterId


@pytest.mark.usefixtures("serial_client")
class TestSystemControlCommands:

    def test_system_reset(self, serial_client: OrbitClient):
        # This test currently fails with a USB disconnect error...
        boot_count_prev = serial_client.parameter.get(ParameterId.BootCount)
        time.sleep(1.0)
        serial_client.system_reset()
        time.sleep(5.0)
        boot_count_new = serial_client.parameter.get(ParameterId.BootCount)
        assert (boot_count_new - boot_count_prev) == 1

    def test_activity_led_rate_change(self, serial_client: OrbitClient):
        test_values = np.arange(0.1, 3.0, 0.25)
        for value in test_values:
            serial_client.set_activity_led_blink_scaler(value)
            time.sleep(0.5)
            new_rate = serial_client.parameter.get(ParameterId.ActivityLedScaler)
            assert math.isclose(new_rate, value, rel_tol=0.00001)

        serial_client.set_activity_led_blink_scaler(1.0)
