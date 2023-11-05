# **********************************************************************************************************************
#   FileName:
#       usb.py
#
#   Description:
#       USB utilities for interacting with the OrbitESC device
#
#   11/05/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import pyudev
from loguru import logger
from typing import Optional


def get_esc_usb_path(serial: str, vendor: str = "cafe") -> Optional[str]:
    """
    Args:
        serial: Serial number of the device to find
        vendor: Vendor ID of the device to find

    Returns:
        Path to the OrbitESC device on the USB bus
    """
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        if (device.get("ID_VENDOR_ID") == vendor) and (device.get("ID_SERIAL_SHORT") == serial):
            return device.device_node

    logger.debug(f"Could not find a device with vendor ID {vendor} and serial {serial}")
