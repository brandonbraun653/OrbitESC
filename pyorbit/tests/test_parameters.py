
import pytest
from pyorbit.serial_client import SerialClient
from pyorbit.serial_messages import MessageId, MessageSubId, ParameterId, ParameterType, StatusCode


@pytest.fixture()
def serial_client() -> SerialClient:
    client = SerialClient(port="/dev/ttyUSB0", baudrate=2000000)
    yield client
    client.close()


class TestReadOnlyParameters:

    def test_get_invalid_parameter(self, serial_client: SerialClient):
        invalid_value = serial_client.get_parameter(ParameterId.Invalid)
        assert invalid_value is None

    def test_get_boot_count(self, serial_client: SerialClient):
        boot_count = serial_client.get_parameter(ParameterId.BootCount)
        assert isinstance(boot_count, int)
        assert boot_count > 0

    def test_get_hw_version(self, serial_client: SerialClient):
        hw_version = serial_client.get_parameter(ParameterId.HwVersion)
        assert isinstance(hw_version, int)
        assert hw_version > 0

    def test_get_sw_version(self, serial_client: SerialClient):
        sw_version = serial_client.get_parameter(ParameterId.SwVersion)
        assert isinstance(sw_version, str)

    def test_get_device_id(self, serial_client: SerialClient):
        device_id = serial_client.get_parameter(ParameterId.DeviceId)
        assert isinstance(device_id, int)
        assert device_id > 0

    def test_get_device_board_name(self, serial_client: SerialClient):
        board_name = serial_client.get_parameter(ParameterId.BoardName)
        assert isinstance(board_name, str)
        assert board_name == "OrbitESC"

    def test_get_device_description(self, serial_client: SerialClient):
        description = serial_client.get_parameter(ParameterId.Description)
        assert isinstance(description, str)
        assert description == "BLDC Motor Controller"
