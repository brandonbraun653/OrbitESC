import sys

import pytest
from loguru import logger
from pyorbit.serial.client import SerialClient
from pyorbit.motor_control import MotorControl
from pyorbit.utility.usb import get_esc_usb_path

_test_device_serial = "DEADBEEF"


@pytest.fixture
def serial_client() -> SerialClient:
    logger.remove()
    logger.add(sys.stderr, level="WARNING")
    client = SerialClient(port=get_esc_usb_path(_test_device_serial), baudrate=2000000)
    yield client
    client.close()


@pytest.fixture
def motor_control(serial_client) -> MotorControl:
    controller = MotorControl(serial_client)
    yield controller
    controller.close()
