# **********************************************************************************************************************
#   FileName:
#       serial_client.py
#
#   Description:
#       Client to connect with an OrbitESC device over serial
#
#   01/20/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from __future__ import annotations

import atexit
import time
from functools import wraps, cmp_to_key
from loguru import logger
from typing import Any, Callable, Dict, List, Union
from pyorbit.serial_pipe import SerialPipe
from pyorbit.exceptions import NotOnlineException
from pyorbit.serial_messages import BaseMessage, PingMessage, SystemTick, ConsoleMessage
from pyorbit.observer import MessageObserver
from threading import Event, Thread, Lock
from queue import Queue


def online_checker(method: Callable) -> Any:
    """
    Decorator to check that the OrbitESC node is online before allowing a method
    call to proceed. This prevents duplicate code on a variety of methods.

    Args:
        method: Function to invoke

    Returns:
        Whatever the method to invoke returns

    References:
        https://stackoverflow.com/a/36944992/8341975
    """

    @wraps(method)
    def _impl(self: SerialClient, *method_args, **method_kwargs):
        # Ensure this decorator is only used on OrbitESC classes
        assert isinstance(self, SerialClient)
        if not self.is_online:
            raise NotOnlineException()

        # Execute the function as normal
        method_output = method(self, *method_args, **method_kwargs)
        return method_output

    return _impl


class ConsoleObserver(MessageObserver):
    """ Simple observer for listening to console messages off the OrbitESC debug port """

    class FrameBuffer:
        def __init__(self):
            self.start_time = time.time()
            self.total_frames = 0
            self.frames = []    # type: List[ConsoleMessage]

        def print_message(self) -> None:
            self.frames = sorted(self.frames, key=cmp_to_key(lambda x1, x2: x1.frame_number - x2.frame_number))
            final_msg = ""
            for frame in self.frames:
                final_msg = final_msg + frame.data.decode("utf-8")

            logger.info(final_msg.strip('\n'))

    def __init__(self):
        super().__init__(func=self._frame_accumulator, msg_type=ConsoleMessage)
        self._frame_lock = Lock()
        self._in_progress_frames = {}   # type: Dict[int, ConsoleObserver.FrameBuffer]
        self._processing_thread = Thread(target=self._frame_processor, name="FrameProcessor")
        self._processing_thread.start()

    def _frame_accumulator(self, msg: ConsoleMessage) -> None:
        """
        Accumulates new frames in to the frame buffer tracking
        Args:
            msg: Message to accumulate

        Returns:
            None
        """
        with self._frame_lock:
            # Start a new tracking entry
            if msg.uuid not in self._in_progress_frames.keys():
                self._in_progress_frames[msg.uuid] = self.FrameBuffer()
                self._in_progress_frames[msg.uuid].total_frames = msg.total_frames
                logger.trace(f"Added start of new message {msg.uuid}")

            # Insert the new message
            self._in_progress_frames[msg.uuid].frames.append(msg)
            logger.trace(f"Added frame {msg.frame_number} for message {msg.uuid}")

    def _frame_processor(self) -> None:
        """
        Runtime processing for printing messages from the console
        Returns:
            None
        """
        while True:
            # Yield time quantum to other threads
            time.sleep(0)

            # Process all frames
            with self._frame_lock:
                uuid_delete_list = []
                for uuid, tracker in self._in_progress_frames.items():
                    # Delete messages that haven't accumulated enough frames in time
                    if len(tracker.frames) != tracker.total_frames and ((time.time() - tracker.start_time) > 30.0):
                        uuid_delete_list.append(uuid)
                        logger.warning(f"Deleting message {uuid}. Not enough frames to reconstruct packet.")

                    # Publish messages that are complete
                    if len(tracker.frames) == tracker.total_frames:
                        tracker.print_message()
                        uuid_delete_list.append(uuid)

                # Handle deleting any stale frames
                for uuid in uuid_delete_list:
                    self._in_progress_frames.pop(uuid)


class SerialClient:
    """ High level serial client to connect with the debug server running on OrbitESC """

    def __init__(self, port: str, baudrate: int):
        """
        Args:
            port: Serial port endpoint to connect with
            baudrate: Desired communication baudrate

        Returns:
            None
        """
        self._transport = SerialPipe()
        self._transport.open(port=port, baudrate=baudrate)
        self._online = False
        self._time_last_online = 0
        self._last_tick = 0

        # Start up the background thread
        self._kill_signal = Event()
        self._notify_signal = Event()
        self._thread = Thread(target=self._background_thread, daemon=True, name=f"SerialClient_Background")
        self._thread.start()

        # Register known observers
        self.com_pipe.subscribe_observer(MessageObserver(func=self._observer_esc_tick, msg_type=SystemTick))
        self.com_pipe.subscribe_observer(ConsoleObserver())

        atexit.register(self._teardown)

    @property
    def com_pipe(self) -> SerialPipe:
        return self._transport

    @property
    def is_online(self) -> bool:
        return self._online

    def put(self, data: bytes) -> None:
        """
        Enqueues a new message for transmission
        Args:
            data: Byte data to transmit

        Returns:
            None
        """
        self.com_pipe.put(data)

    def _teardown(self) -> None:
        self._kill_signal.set()
        self._thread.join() if self._thread.is_alive() else None

        self._transport.close()

    def _background_thread(self) -> None:
        """
        General purpose thread to handle background tasks that help with processing the ESC data
        Returns:
            None
        """
        while not self._kill_signal.is_set():
            self._notify_signal.wait(timeout=0.1)

            # Do a tick timeout check to ensure the node hasn't dropped off the face of the earth
            if self._online and ((time.time() - self._time_last_online) > 2.0):
                logger.warning("Serial link is offline")
                self._online = False

    def _observer_esc_tick(self, msg: SystemTick) -> None:
        """
        Looks for the SystemTick of the registered node to determine online/offline status
        Args:
            msg: Received CAN bus message

        Returns:
            None
        """
        self._time_last_online = time.time()
        self._last_tick = msg.tick
        if not self._online:
            logger.info("Serial link is online")
            self._online = True


if __name__ == "__main__":
    from pyorbit.serial_messages import PingMessage
    ping = PingMessage()
    client = SerialClient(port="/dev/ttyUSB0", baudrate=921600)
    # pipe.put(ping.serialize())
    time.sleep(500)
