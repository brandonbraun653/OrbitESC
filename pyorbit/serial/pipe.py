# **********************************************************************************************************************
#   FileName:
#       pipe.py
#
#   Description:
#       Serial pipe framing formatter to communicate with an OrbitESC device
#
#   12/12/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************
import queue
import time
from typing import List, Optional, Union

import pyorbit.nanopb.serial_interface_pb2 as proto
import google.protobuf.message as g_proto_msg
from cobs import cobs
from loguru import logger
from serial import Serial
from threading import Thread, Event
from queue import Queue
from pyorbit.publisher import Publisher
from pyorbit.serial.messages import BaseMessage
from pyorbit.serial.observers import ResponseObserver
from pyorbit.serial.parameters import MessageTypeMap


class SerialPipePublisher(Publisher):
    """
    Message server for RTX-ing COBS encoded protocol buffer messages over a serial connection. Implements
    the Publisher interface for message dispatching to observers.
    """

    def __init__(self):
        super().__init__()
        self._serial = Serial(timeout=5)
        self._kill_event = Event()

        # TX resources
        self._tx_thread = Thread()
        self._tx_msgs: Queue[bytes] = Queue()

        # RX resources
        self._rx_thread = Thread()
        self._rx_msgs = Queue()
        self._rx_frames: List[bytes] = []
        self._rx_byte_buffer = bytearray()

        # Dispatch resources
        self._dispatch_thread = Thread()

    def open(self, port: str, baudrate: int) -> None:
        """
        Opens a serial port for communication
        Args:
            port: Serial port endpoint to connect with
            baudrate: Desired communication baudrate

        Returns:
            None
        """
        # Clear memory
        self._rx_msgs = Queue()
        self._rx_frames = []

        # Open the serial port with the desired configuration
        self._serial.port = port
        self._serial.baudrate = baudrate
        self._serial.exclusive = True
        self._serial.open()
        logger.trace(f"Opened serial port {port} at {baudrate} baud")

        # Spawn the IO threads for enabling communication
        self._kill_event.clear()
        self._rx_thread = Thread(target=self._rx_decoder_thread, name="RXDecoder")
        self._rx_thread.start()
        self._tx_thread = Thread(target=self._tx_encoder_thread, name="TXEncoder")
        self._tx_thread.start()
        self._dispatch_thread = Thread(target=self._rx_dispatcher_thread, name="RXDispatch")
        self._dispatch_thread.start()

    def close(self) -> None:
        """
        Closes the pipe, destroying all resources
        Returns:
            None
        """
        # Kill the threads first since they consume the serial port
        self._kill_event.set()
        self._rx_thread.join() if self._rx_thread.is_alive() else None
        self._tx_thread.join() if self._tx_thread.is_alive() else None
        self._dispatch_thread.join() if self._dispatch_thread.is_alive() else None

        # Push any remaining information and then close out resources
        if self._serial.is_open:
            self._serial.flush()
            self._serial.close()

    def write(self, data: bytes) -> None:
        """
        Enqueues a new message for transmission
        Args:
            data: Byte data to transmit

        Returns:
            None
        """
        self._tx_msgs.put(data, block=True)

    def write_and_wait(self, msg: BaseMessage, timeout: Union[int, float]) -> Optional[BaseMessage]:
        """
        Writes a message to the serial pipe and waits for a response
        Args:
            msg: Message to send
            timeout: Time to wait for a response

        Returns:
            Response message
        """
        observer = ResponseObserver(txn_uuid=msg.uuid, timeout=timeout)
        sub_id = self.subscribe_observer(observer)

        # Send the message and wait for the response
        self.write(msg.serialize())
        result = observer.wait()

        # Clean up the observer
        self.unsubscribe(sub_id)
        return result

    def _tx_encoder_thread(self) -> None:
        """
        Encodes queued TX packets with COBS framing and sends it on the wire
        Returns:
            None
        """
        logger.trace(f"Starting OrbitESC internal Serial message encoder thread")
        while not self._kill_event.is_set():
            # Pull the latest data off the queue
            try:
                raw_frame = self._tx_msgs.get(block=True, timeout=0.1)
            except queue.Empty:
                continue

            # Encode the frame w/termination byte, then transmit
            encoded_frame = cobs.encode(raw_frame) + b'\x00'
            self._serial.write(encoded_frame)
            logger.trace(f"Write {len(encoded_frame)} bytes: {repr(encoded_frame)}")

        logger.trace("Terminating OrbitESC internal Serial message encoder")

    def _rx_decoder_thread(self) -> None:
        """
        Thread to handle reception of data from the connected endpoint, encoded with COBS framing.
        Will parse valid COBS packets into the appropriate protocol buffer message type.

        Returns:
            None
        """
        logger.trace("Starting OrbitESC internal Serial message decoder thread")
        while not self._kill_event.is_set():
            # Allow other threads time to execute
            time.sleep(0.001)

            # Fill the cache with the raw data from the bus
            if new_data := self._serial.read_all():
                self._rx_byte_buffer.extend(new_data)
                logger.trace(f"Received {len(new_data)} bytes: {new_data}")
            elif not len(self._rx_byte_buffer):
                continue

            # Parse the data in the cache to extract all waiting COBS frames
            self._rx_decode_available_cobs_frames()

            # Parse each decoded frame into a higher level message type
            for frame in self._rx_frames:
                if full_msg := self._decode_pb_frame(frame):
                    self._rx_msgs.put(full_msg)
                    logger.trace(f"Received message type {full_msg.name}. UUID: {full_msg.uuid}")

        logger.trace("Terminating OrbitESC internal Serial message decoder")

    def _rx_dispatcher_thread(self) -> None:
        """
        Takes new messages and dispatches them to all observers
        Returns:
            None
        """
        logger.trace("Starting OrbitESC internal Serial message dispatcher thread")
        while not self._kill_event.is_set():
            self.prune_expired_observers()
            try:
                while msg := self._rx_msgs.get(block=True, timeout=0.01):
                    self.accept(msg)
            except queue.Empty:
                continue

        logger.trace("Terminating OrbitESC internal Serial message dispatcher")

    def _rx_decode_available_cobs_frames(self) -> None:
        """
        Parses the current RX buffer for any available COBS frames and decodes them into
        protocol buffer messages.

        Returns:
            None
        """
        try:
            while True:
                # Search for the frame delimiter and extract an entire frame if it exists
                eof_idx = self._rx_byte_buffer.index(b'\x00')
                cobs_frame = self._rx_byte_buffer[:eof_idx]

                # Remove the frame from the buffer by slicing it out
                self._rx_byte_buffer = self._rx_byte_buffer[eof_idx + 1:]

                # Decode the message and store it in our RX buffer
                if decoded_frame := self._decode_cobs_frame(cobs_frame):
                    self._rx_frames.append(decoded_frame)

        except ValueError:
            # No more frames available
            pass

    @staticmethod
    def _decode_cobs_frame(frame: bytes) -> Optional[bytes]:
        """
        Decodes a COBS encoded frame into the original message
        Args:
            frame: COBS encoded frame

        Returns:
            Decoded message
        """
        try:
            return cobs.decode(frame)
        except cobs.DecodeError:
            # Nothing much to do here if this fails. Just move on to the next frame.
            logger.trace("Failed to decode COBS frame. Likely partially received message.")
            return None

    @staticmethod
    def _decode_pb_frame(frame: bytes) -> Optional[proto.BaseMessage]:
        """
        Decodes a COBS encoded frame into a protocol buffer message
        Args:
            frame: COBS encoded frame

        Returns:
            Decoded protocol buffer message
        """
        # Peek the header of the message
        try:
            base_msg = proto.BaseMessage()
            base_msg.ParseFromString(frame)
            if base_msg.header.msgId not in MessageTypeMap.keys():
                logger.warning(f"Unsupported message ID: {base_msg.header.msgId}")
                return None
        except g_proto_msg.DecodeError:
            logger.trace("Frame did not contain the expected header. Unable to parse.")
            return None

        # Now do the full decode since the claimed type is supported
        full_msg = MessageTypeMap[base_msg.header.msgId]()
        try:
            full_msg.deserialize(frame)
            return full_msg
        except g_proto_msg.DecodeError:
            logger.error(f"Failed to decode {full_msg.name} type")
            return None
