import pytest
from pyorbit.serial_client import SerialClient


@pytest.fixture()
def serial_client() -> SerialClient:
    client = SerialClient(port="/dev/ttyUSB0", baudrate=2000000)
    yield client
    client.close()
