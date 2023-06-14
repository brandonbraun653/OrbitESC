import pytest
from pyorbit.serial.client import SerialClient
from pyorbit.motor_control import MotorControl


@pytest.fixture
def serial_client() -> SerialClient:
    client = SerialClient(port="/dev/ttyUSB0", baudrate=2000000)
    yield client
    client.close()


@pytest.fixture
def motor_control(serial_client) -> MotorControl:
    controller = MotorControl(serial_client)
    yield controller
    controller.close()
