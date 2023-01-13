# **********************************************************************************************************************
#   FileName:
#       serial_pipe.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   12/12/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import atexit
import numpy as np
from serial import Serial
from threading import Thread, Event
from queue import Queue
from numpy_ringbuffer import RingBuffer


class SerialPipe:
    """ Message server for RTX-ing Protocol Buffer over a serial connection """

    def __init__(self):
        self._serial = Serial(timeout=5)
        self._rx_kill = Event()
        self._rx_thread = Thread()
        self._tx_kill = Event()
        self._tx_thread = Thread()
        self._rx_msgs = Queue()
        self._tx_msgs = Queue()

    def open(self, port: str, baudrate: int = 921600):
        """

        Args:
            port: Serial port endpoint to connect with
            baudrate: Desired communication baudrate

        Returns:
            None
        """
        # Open the serial port with the desired configuration
        self._serial.port = port
        self._serial.baudrate = baudrate
        self._serial.open()

        # Spawn the IO threads for enabling communication
        self._rx_kill.clear()
        self._rx_thread = Thread(target=self._rx_decoder, name="RX", daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        """
        Closes the pipe, destroying all resources
        Returns:
            None
        """
        self._serial.close()
        self._rx_kill.set()
        self._rx_thread.join()

    @atexit.register
    def _teardown(self):
        self._serial.close()

    def _rx_decoder(self):
        """
        Thread to handle reception of data from the connected endpoint, encoded
        with COBS framing. Will parse valid COBS packets into the appropriate
        protocol buffer message type.

        Returns:
            None
        """
        rx_byte_buffer = RingBuffer(capacity=1024, dtype=bytes)

        while not self._rx_kill.is_set():
            # Allow other threads time to execute
            yield
            if not self._serial.is_open():
                continue

            # Fill the cache with the raw data from the bus
            new_data = self._serial.read_all()
            if new_data:
                rx_byte_buffer.extend(new_data)

            # Parse the data in the cache to extract COBS frames


if __name__ == "__main__":
    pass
        