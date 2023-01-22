# **********************************************************************************************************************
#   FileName:
#       serial_pipe_framing.py
#
#   Description:
#       Serial pipe framing formatter to communicate with an OrbitESC device
#
#   12/12/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import queue
import time
import atexit
import pyorbit.nanopb.serial_interface_pb2 as proto
import google.protobuf.message as g_proto_msg
from cobs import cobs
from loguru import logger
from pyorbit.serial_messages import MessageId, MessageTypeMap
from serial import Serial
from threading import Thread, Event
from queue import Queue


class COBSFramer:
    """ Message server for RTX-ing COBS encoded messages over a serial connection """

    def __init__(self):
        self._serial = Serial(timeout=5)
        self._thread_kill = Event()
        self._rx_thread = Thread()
        self._tx_thread = Thread()
        self._rx_msgs = Queue()
        self._tx_msgs = Queue()

    def open(self, port: str, baudrate: int):
        """
        Opens a serial port for communication
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
        self._thread_kill.clear()
        self._rx_thread = Thread(target=self._rx_decoder, name="RX")
        self._rx_thread.start()
        self._tx_thread = Thread(target=self._tx_encoder, name="TX")
        self._tx_thread.start()

    def close(self) -> None:
        """
        Closes the pipe, destroying all resources
        Returns:
            None
        """
        # Kill the threads first since they consume the serial port
        self._thread_kill.set()
        self._rx_thread.join() if self._rx_thread.is_alive() else None
        self._tx_thread.join() if self._tx_thread.is_alive() else None

        # Push any remaining information and then close out resources
        if self._serial.is_open:
            self._serial.flush()
            self._serial.close()

    def get(self) -> bytes:
        """
        Returns:
            Latest message received on the pipe
        """
        try:
            return self._rx_msgs.get(block=False)
        except queue.Empty:
            return bytes()

    def put(self, data: bytes) -> None:
        """
        Enqueues a new message for transmission
        Args:
            data: Byte data to transmit

        Returns:
            None
        """
        self._tx_msgs.put(data, block=True)

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
        rx_byte_buffer = bytearray()

        while not self._thread_kill.is_set():
            # Allow other threads time to execute
            time.sleep(0)

            if not self._serial.is_open:
                continue

            # Fill the cache with the raw data from the bus
            new_data = self._serial.read_all()
            if new_data:
                rx_byte_buffer.extend(new_data)

            # Parse the data in the cache to extract all waiting COBS frames
            frames_available = True
            frame_list = []
            while frames_available:
                try:
                    # Search for the frame delimiter and extract an entire frame if it exists
                    end_of_frame_idx = rx_byte_buffer.index(b'\x00')
                    encoded_frame = rx_byte_buffer[:end_of_frame_idx]
                    rx_byte_buffer = rx_byte_buffer[end_of_frame_idx+1:]

                    # Decode the message and store it in our RX buffer
                    try:
                        decoded_frame = cobs.decode(encoded_frame)
                        frame_list.append(decoded_frame)
                    except cobs.DecodeError:
                        # Data frame misaligned most likely
                        logger.trace("Partial COBS frame received")
                except ValueError:
                    # Frame delimiter byte was not found
                    frames_available = False

            # Parse each decoded frame into a higher level message type
            for frame in frame_list:
                # Peek the header of the message
                try:
                    base_msg = proto.BaseMessage()
                    base_msg.ParseFromString(frame)
                    if base_msg.header.msgId not in MessageTypeMap.keys():
                        logger.warning(f"Unsupported message ID: {base_msg.header.msgId}")
                        continue
                except g_proto_msg.DecodeError:
                    logger.error("Frame did not contain the expected header. Unable to parse.")
                    continue

                # Now do the full decode since the claimed type is supported
                full_msg = MessageTypeMap[base_msg.header.msgId]()
                try:
                    full_msg.deserialize(frame)
                except g_proto_msg.DecodeError:
                    logger.error(f"Failed to decode {full_msg.name} type")
                    continue

                # Now push the completed message onto the queue for someone else to handle
                self._rx_msgs.put(full_msg)

    def _tx_encoder(self):
        """
        Encodes queued TX packets with COBS framing and sends it on the wire
        Returns:
            None
        """
        while not self._thread_kill.is_set():
            # Pull the latest data off the queue
            try:
                raw_frame = self._tx_msgs.get(block=True, timeout=0.1)  # type: bytes
            except queue.Empty:
                continue

            # Encode the frame w/termination byte, then transmit
            encoded_frame = cobs.encode(raw_frame) + b'\x00'
            logger.trace(f"Write {len(encoded_frame)} bytes: {repr(encoded_frame)}")
            self._serial.write(encoded_frame)


if __name__ == "__main__":
    from pyorbit.serial_messages import PingMessage
    ping = PingMessage()
    pipe = COBSFramer()
    pipe.open(port="/dev/ttyUSB0", baudrate=921600)
    pipe.put(ping.serialize())
    time.sleep(1)
    pipe.close()
