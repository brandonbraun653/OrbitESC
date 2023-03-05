import math
from pyorbit.tests.fixtures import *
from pyorbit.serial_client import SerialClient
from pyorbit.serial_messages import ParameterId
from pyorbit.can_messages import NodeID


@pytest.mark.usefixtures("serial_client")
class TestReadOnlyParameters:

    def test_get_invalid_parameter(self, serial_client: SerialClient):
        invalid_value = serial_client.parameter.get(ParameterId.Invalid)
        assert invalid_value is None

    def test_get_boot_count(self, serial_client: SerialClient):
        boot_count = serial_client.parameter.get(ParameterId.BootCount)
        assert isinstance(boot_count, int)
        assert boot_count > 0

    def test_get_hw_version(self, serial_client: SerialClient):
        hw_version = serial_client.parameter.get(ParameterId.HwVersion)
        assert isinstance(hw_version, int)
        assert hw_version > 0

    def test_get_sw_version(self, serial_client: SerialClient):
        sw_version = serial_client.parameter.get(ParameterId.SwVersion)
        assert isinstance(sw_version, str)

    def test_get_device_id(self, serial_client: SerialClient):
        device_id = serial_client.parameter.get(ParameterId.DeviceId)
        assert isinstance(device_id, int)
        assert device_id > 0

    def test_get_device_board_name(self, serial_client: SerialClient):
        board_name = serial_client.parameter.get(ParameterId.BoardName)
        assert isinstance(board_name, str)
        assert board_name == "OrbitESC"

    def test_get_device_description(self, serial_client: SerialClient):
        description = serial_client.parameter.get(ParameterId.Description)
        assert isinstance(description, str)
        assert description == "BLDC Motor Controller"

    def test_get_boot_mode(self, serial_client: SerialClient):
        boot_mode = serial_client.parameter.get(ParameterId.BootMode)
        assert isinstance(boot_mode, int)
        assert boot_mode >= 0


@pytest.mark.usefixtures("serial_client")
class TestSerialNumber:
    NORMAL_SERIAL_NUMBER = "123456"

    def test_assign_normal_serial_number(self, serial_client: SerialClient):
        assert serial_client.parameter.set(ParameterId.SerialNumber, self.NORMAL_SERIAL_NUMBER)
        assert serial_client.parameter.get(ParameterId.SerialNumber) == self.NORMAL_SERIAL_NUMBER

    def test_assign_empty_serial_number(self, serial_client: SerialClient):
        assert serial_client.parameter.set(ParameterId.SerialNumber, "")
        assert serial_client.parameter.get(ParameterId.SerialNumber) == ""

    def test_assign_too_long_serial_number(self, serial_client: SerialClient):
        # Assign a valid serial number
        self.test_assign_normal_serial_number(serial_client)

        # Assign a too long serial number and validate it doesn't update
        assert not serial_client.parameter.set(ParameterId.SerialNumber, "123456789")
        assert serial_client.parameter.get(ParameterId.SerialNumber) == self.NORMAL_SERIAL_NUMBER


@pytest.mark.usefixtures("serial_client")
class TestCANNodeId:

    def test_assign_valid_can_id(self, serial_client: SerialClient):
        valid_ids = [NodeID.NODE_0, NodeID.NODE_1, NodeID.NODE_2, NodeID.NODE_3]
        for node_id in valid_ids:
            assert serial_client.parameter.set(ParameterId.CanNodeId, node_id)
            assert serial_client.parameter.get(ParameterId.CanNodeId) == node_id

    def test_assign_invalid_can_id(self, serial_client: SerialClient):
        invalid_ids = [NodeID.NODE_PC, 15]

        # Assign a valid CAN ID first
        assert serial_client.parameter.set(ParameterId.CanNodeId, NodeID.NODE_0)

        # Validate the invalid IDs don't update
        for node_id in invalid_ids:
            assert not serial_client.parameter.set(ParameterId.CanNodeId, node_id)
            assert serial_client.parameter.get(ParameterId.CanNodeId) == NodeID.NODE_0


@pytest.mark.usefixtures("serial_client")
class TestMotorControlParameters:

    def test_stator_pwm_freq(self, serial_client: SerialClient):
        assert serial_client.parameter.set(ParameterId.StatorPWMFrequency, 10000)
        assert serial_client.parameter.get(ParameterId.StatorPWMFrequency) == 10000

    def test_speed_control_freq(self, serial_client: SerialClient):
        assert serial_client.parameter.set(ParameterId.SpeedControlFrequency, 1000)
        assert serial_client.parameter.get(ParameterId.SpeedControlFrequency) == 1000

    def test_target_idle_rpm(self, serial_client: SerialClient):
        test_values = [1000.0, 1500.0, 2000.0, 500.0, 1000.0]

        # TODO: Setting value 500 doesn't work. Returns nothing in the data field.

        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.TargetIdleRPM, setting)
            assert math.isclose(serial_client.parameter.get(ParameterId.TargetIdleRPM), setting, abs_tol=0.0001)

    def test_speed_control_pid(self, serial_client: SerialClient):
        test_values = [[0.1, 0.2, 0.3], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]

        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.SpeedControlKp, setting[0])
            assert serial_client.parameter.set(ParameterId.SpeedControlKi, setting[1])
            assert serial_client.parameter.set(ParameterId.SpeedControlKd, setting[2])
            assert math.isclose(serial_client.parameter.get(ParameterId.SpeedControlKp), setting[0], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.SpeedControlKi), setting[1], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.SpeedControlKd), setting[2], abs_tol=0.0001)

    def test_current_control_pid(self, serial_client: SerialClient):
        test_values = [[0.1, 0.2, 0.3], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]

        # TODO: Some values aren't getting applied

        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.CurrentControlQKp, setting[0])
            assert serial_client.parameter.set(ParameterId.CurrentControlQKi, setting[1])
            assert serial_client.parameter.set(ParameterId.CurrentControlQKd, setting[2])
            assert serial_client.parameter.set(ParameterId.CurrentControlDKp, setting[0])
            assert serial_client.parameter.set(ParameterId.CurrentControlDKi, setting[1])
            assert serial_client.parameter.set(ParameterId.CurrentControlDKd, setting[2])
            assert math.isclose(serial_client.parameter.get(ParameterId.CurrentControlQKp), setting[0], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.CurrentControlQKi), setting[1], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.CurrentControlQKd), setting[2], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.CurrentControlDKp), setting[0], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.CurrentControlDKi), setting[1], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.CurrentControlDKd), setting[2], abs_tol=0.0001)
