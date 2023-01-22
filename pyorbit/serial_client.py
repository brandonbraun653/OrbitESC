# **********************************************************************************************************************
#   FileName:
#       serial_pipe_framing.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   01/20/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import atexit
import time
from typing import Any
from pyorbit.serial_pipe_framing import COBSFramer
from pyorbit.serial_messages import PingMessage
from threading import Event, Thread
from queue import Queue


class SerialPipe:

    def __init__(self):
        self._transport = COBSFramer()
        atexit.register(self._teardown)

    def _teardown(self) -> None:
        self._transport.close()

    def open(self, port: str, baudrate: int):
        """
        Opens a serial port for communication
        Args:
            port: Serial port endpoint to connect with
            baudrate: Desired communication baudrate

        Returns:
            None
        """
        self._transport.open(port=port, baudrate=baudrate)

    def close(self) -> None:
        self._teardown()

    def put(self, ) -> None:
        self._transport.put(message.SerializeToString())

    def get(self) -> Message:
        pass


if __name__ == "__main__":
    ping = PingMessage()
    pipe = SerialPipe()
    pipe.open(port="/dev/ttyUSB0", baudrate=921600)
    pipe.put(ping.pb_message)
    time.sleep(1)
    pipe.close()
