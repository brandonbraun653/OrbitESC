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
        valid_ids = [NodeID.NODE_PC, NodeID.NODE_1, NodeID.NODE_2, NodeID.NODE_3]
        for node_id in valid_ids:
            assert serial_client.parameter.set(ParameterId.CanNodeId, node_id)
            assert serial_client.parameter.get(ParameterId.CanNodeId) == node_id
