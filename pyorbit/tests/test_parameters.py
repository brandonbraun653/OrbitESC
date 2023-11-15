import math
from pyorbit.tests.fixtures import *
from pyorbit.serial.client import OrbitClient
from pyorbit.serial.parameters import ParameterId
from pyorbit.can_messages import NodeID


@pytest.mark.usefixtures("serial_client")
class TestReadOnlyParameters:

    def test_get_invalid_parameter(self, serial_client: OrbitClient):
        invalid_value = serial_client.parameter.get(ParameterId.Invalid)
        assert invalid_value is None

    def test_get_boot_count(self, serial_client: OrbitClient):
        boot_count = serial_client.parameter.get(ParameterId.BootCount)
        assert isinstance(boot_count, int)
        assert boot_count > 0

    def test_get_hw_version(self, serial_client: OrbitClient):
        hw_version = serial_client.parameter.get(ParameterId.HwVersion)
        assert isinstance(hw_version, int)
        assert hw_version > 0

    def test_get_sw_version(self, serial_client: OrbitClient):
        sw_version = serial_client.parameter.get(ParameterId.SwVersion)
        assert isinstance(sw_version, str)

    def test_get_device_id(self, serial_client: OrbitClient):
        device_id = serial_client.parameter.get(ParameterId.DeviceId)
        assert isinstance(device_id, int)
        assert device_id > 0

    def test_get_device_board_name(self, serial_client: OrbitClient):
        board_name = serial_client.parameter.get(ParameterId.BoardName)
        assert isinstance(board_name, str)
        assert board_name == "OrbitESC"

    def test_get_device_description(self, serial_client: OrbitClient):
        description = serial_client.parameter.get(ParameterId.Description)
        assert isinstance(description, str)
        assert description == "BLDC Motor Controller"


@pytest.mark.usefixtures("serial_client")
class TestSerialNumber:
    NORMAL_SERIAL_NUMBER = "123456"

    def test_assign_normal_serial_number(self, serial_client: OrbitClient):
        assert serial_client.parameter.set(ParameterId.SerialNumber, self.NORMAL_SERIAL_NUMBER)
        assert serial_client.parameter.get(ParameterId.SerialNumber) == self.NORMAL_SERIAL_NUMBER

    def test_assign_empty_serial_number(self, serial_client: OrbitClient):
        assert serial_client.parameter.set(ParameterId.SerialNumber, "")
        assert serial_client.parameter.get(ParameterId.SerialNumber) == ""

    def test_assign_too_long_serial_number(self, serial_client: OrbitClient):
        # Assign a valid serial number
        self.test_assign_normal_serial_number(serial_client)

        # Assign a too long serial number and validate it doesn't update
        assert not serial_client.parameter.set(ParameterId.SerialNumber, "123456789")
        assert serial_client.parameter.get(ParameterId.SerialNumber) == self.NORMAL_SERIAL_NUMBER


@pytest.mark.usefixtures("serial_client")
class TestCANNodeId:

    def test_assign_valid_can_id(self, serial_client: OrbitClient):
        valid_ids = [NodeID.NODE_0, NodeID.NODE_1, NodeID.NODE_2, NodeID.NODE_3]
        for node_id in valid_ids:
            assert serial_client.parameter.set(ParameterId.CanNodeId, node_id)
            assert serial_client.parameter.get(ParameterId.CanNodeId) == node_id

    def test_assign_invalid_can_id(self, serial_client: OrbitClient):
        invalid_ids = [NodeID.NODE_PC, 15]

        # Assign a valid CAN ID first
        assert serial_client.parameter.set(ParameterId.CanNodeId, NodeID.NODE_0)

        # Validate the invalid IDs don't update
        for node_id in invalid_ids:
            assert not serial_client.parameter.set(ParameterId.CanNodeId, node_id)
            assert serial_client.parameter.get(ParameterId.CanNodeId) == NodeID.NODE_0


@pytest.mark.usefixtures("serial_client")
class TestMotorControlParameters:

    def test_stator_pwm_freq(self, serial_client: OrbitClient):
        assert serial_client.parameter.set(ParameterId.StatorPWMFrequency, 10000)
        assert serial_client.parameter.get(ParameterId.StatorPWMFrequency) == 10000

    def test_speed_control_freq(self, serial_client: OrbitClient):
        assert serial_client.parameter.set(ParameterId.SpeedControlFrequency, 1000)
        assert serial_client.parameter.get(ParameterId.SpeedControlFrequency) == 1000

    def test_target_idle_rpm(self, serial_client: OrbitClient):
        test_values = [1000.0, 1500.0, 2000.0, 500.0, 1000.0]

        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.TargetIdleRPM, setting)
            assert math.isclose(serial_client.parameter.get(ParameterId.TargetIdleRPM), setting, abs_tol=0.0001)

    def test_speed_control_pid(self, serial_client: OrbitClient):
        test_values = [[0.1, 0.2, 0.3], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]

        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.SpeedControlKp, setting[0])
            assert serial_client.parameter.set(ParameterId.SpeedControlKi, setting[1])
            assert serial_client.parameter.set(ParameterId.SpeedControlKd, setting[2])
            assert math.isclose(serial_client.parameter.get(ParameterId.SpeedControlKp), setting[0], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.SpeedControlKi), setting[1], abs_tol=0.0001)
            assert math.isclose(serial_client.parameter.get(ParameterId.SpeedControlKd), setting[2], abs_tol=0.0001)

    def test_current_control_pid(self, serial_client: OrbitClient):
        test_values = [[0.1, 0.2, 0.3], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]

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


@pytest.mark.usefixtures("serial_client")
class TestMotorDescriptorParameters:

    def test_rotor_poles(self, serial_client: OrbitClient):
        pole_values = [4, 6, 8, 10, 12, 14]
        for pole in pole_values:
            assert serial_client.parameter.set(ParameterId.RotorPoles, pole)
            assert serial_client.parameter.get(ParameterId.RotorPoles) == pole

    def test_stator_slots(self, serial_client: OrbitClient):
        slot_values = [8, 16, 32, 12]
        for slot in slot_values:
            assert serial_client.parameter.set(ParameterId.StatorSlots, slot)
            assert serial_client.parameter.get(ParameterId.StatorSlots) == slot

    def test_stator_resistance(self, serial_client: OrbitClient):
        resistance_values = [0.1, 0.2, 0.3, 0.4, 0.5]
        for resistance in resistance_values:
            assert serial_client.parameter.set(ParameterId.StatorResistance, resistance)
            assert math.isclose(serial_client.parameter.get(ParameterId.StatorResistance), resistance, abs_tol=0.0001)

    def test_stator_inductance(self, serial_client: OrbitClient):
        inductance_values = [0.1, 0.2, 0.3, 0.4, 0.5]
        for inductance in inductance_values:
            assert serial_client.parameter.set(ParameterId.StatorInductance, inductance)
            assert math.isclose(serial_client.parameter.get(ParameterId.StatorInductance), inductance, abs_tol=0.0001)


@pytest.mark.usefixtures("serial_client")
class TestMonitorThresholds:

    def test_peak_current_threshold(self, serial_client: OrbitClient):
        test_values = [0.1, 0.9, 10.0, 20.0, 5.0]
        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.PeakCurrentThreshold, setting)
            assert math.isclose(serial_client.parameter.get(ParameterId.PeakCurrentThreshold), setting, abs_tol=0.0001)

    def test_peak_voltage_threshold(self, serial_client: OrbitClient):
        test_values = [0.1, 0.9, 10.0, 20.0, 16.0]
        for setting in test_values:
            assert serial_client.parameter.set(ParameterId.PeakVoltageThreshold, setting)
            assert math.isclose(serial_client.parameter.get(ParameterId.PeakVoltageThreshold), setting, abs_tol=0.0001)


@pytest.mark.usefixtures("serial_client")
class TestSystemControl:

    def test_stream_phase_currents(self, serial_client: OrbitClient):
        assert serial_client.parameter.set(ParameterId.StreamPhaseCurrents, False)
        assert serial_client.parameter.get(ParameterId.StreamPhaseCurrents) is False
        assert serial_client.parameter.set(ParameterId.StreamPhaseCurrents, True)
        assert serial_client.parameter.get(ParameterId.StreamPhaseCurrents) is True
