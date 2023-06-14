import math
import time
import numpy as np
from pyorbit.tests.fixtures import *
from pyorbit.motor_control import MotorControl


@pytest.mark.usefixtures("motor_control")
class TestMotorControl:

    def test_motor_engage_state(self, motor_control: MotorControl):
        assert motor_control.set_power_stage(MotorControl.ActiveState.Enabled)
        assert motor_control.set_power_stage(MotorControl.ActiveState.Disabled)