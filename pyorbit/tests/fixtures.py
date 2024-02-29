import sys

import pytest
from loguru import logger
from pyorbit.serial.client import OrbitClient
from pyorbit.utility.usb import get_esc_usb_path

_test_device_serial = "DEADBEEF"


@pytest.fixture
def serial_client() -> OrbitClient:
    logger.remove()
    logger.add(sys.stderr, level="ERROR")
    # client = OrbitClient(port=get_esc_usb_path(_test_device_serial), baudrate=2000000)
    client = OrbitClient(port=37218)
    yield client
    client.close()
