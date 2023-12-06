# **********************************************************************************************************************
#   FileName:
#       can_pipe.py
#
#   Description:
#       Core driver for interacting with an OrbitESC node over CAN bus
#
#   03/02/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import atexit
import can
import re
import subprocess

from can import BusABC
from ifconfigparser import IfconfigParser
from loguru import logger
from typing import List
from threading import Thread, Event
from pyorbit.publisher import Publisher


class CANPipe(Publisher):
    """ Interface to a number of ESC nodes communicating over CAN bus """

    def __init__(self, can_device: str, bit_rate: int):
        """
        Args:
            can_device: Device to communicate over as listed in ifconfig
            bit_rate: Effective CAN bus communication speed
        """
        super().__init__()

        # CAN related variables
        try:
            self._bus = can.ThreadSafeBus(bustype='socketcan', channel=can_device, bitrate=bit_rate)
        except OSError:
            logger.warning(f"Device {can_device} not found. Creating virtual CAN device.")
            self._bus = can.interface.Bus("virtual_" + can_device, bustype='virtual')

        self._default_reader = can.BufferedReader()
        self._message_notifier = can.Notifier(self._bus, [self._default_reader])

        # Internal thread related variables
        self._kill_event = Event()
        self._reader_thread = Thread(target=self._can_message_handler, daemon=True, name="CANPipe_Reader")

        # Kick things off!
        atexit.register(self.shutdown)
        self._reader_thread.start()

    @classmethod
    def available_interfaces(cls) -> List[str]:
        """
        Returns:
            Available interfaces that match some kind of CAN bus driver
        """
        intf_list = IfconfigParser(subprocess.getoutput("ifconfig -a")).list_interfaces()
        return list(filter(re.compile(r"^can.*").match, intf_list))

    @property
    def bus(self) -> BusABC:
        """
        Returns:
            Instance of the raw CAN bus connection. This can be used to send/receive data on the wire.
        """
        return self._bus

    def shutdown(self) -> None:
        """
        Cleans up resources that make up the ESC communication runtime
        Returns:
            None
        """
        # Kill the CAN bus message reader first to stop the influx of data. Allow some time to process
        # any straggling messages that may be left in the reader queue.
        self._message_notifier.stop()
        self._bus.shutdown()

        # Kill local resources last
        self._kill_event.set()
        self._reader_thread.join()

    def _can_message_handler(self) -> None:
        """
        The OrbitESC internal thread to handle async message responses or other kinds of message
        based actions that need to occur under the hood.

        Returns:
            None
        """
        from pyorbit.can_messages import MessageTypeMap
        logger.debug("Starting OrbitESC internal CAN message dispatcher")

        while not self._kill_event.is_set():
            self.prune_expired_observers()

            # Process any new CAN messages
            msg = self._default_reader.get_message(timeout=0.001)
            if msg and msg.arbitration_id in MessageTypeMap.keys():
                type_instance = MessageTypeMap[msg.arbitration_id]()
                type_instance.unpack(msg)
                self.accept(type_instance)

            # Check for termination of the reader
            if self._default_reader.is_stopped:
                self._kill_event.set()

        logger.debug("Terminating OrbitESC internal CAN message dispatcher")
