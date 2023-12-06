# **********************************************************************************************************************
#   FileName:
#       observers.py
#
#   Description:
#       Various observers for the serial client
#
#   01/20/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from __future__ import annotations

import copy
import time
from functools import cmp_to_key
from queue import Queue

from loguru import logger
from typing import Dict, List, Callable, Optional
from pyorbit.serial.messages import *
from pyorbit.observers import MessageObserver
from threading import Thread, Lock, Event
from pyorbit.serial.messages import BasePBMsg


class ConsoleObserver(MessageObserver):
    """ Simple observer for listening to console messages off the OrbitESC debug port """

    class FrameBuffer:
        def __init__(self):
            self.start_time = time.time()
            self.total_frames = 0
            self.frames: List[ConsolePBMsg] = []

        def message(self) -> str:
            self.frames = sorted(self.frames, key=cmp_to_key(lambda x1, x2: x1.frame_number - x2.frame_number))
            final_msg = ""
            for frame in self.frames:
                final_msg = final_msg + frame.data.decode("utf-8")
            return final_msg

    def __init__(self, on_msg_rx: Callable[[str], None]):
        super().__init__(func=self._frame_accumulator, msg_type=ConsolePBMsg)
        self._frame_lock = Lock()
        self._on_msg_rx = on_msg_rx
        self._in_progress_frames = {}  # type: Dict[int, ConsoleObserver.FrameBuffer]
        self._processing_thread = Thread(target=self._frame_processor, name="FrameProcessor", daemon=True)
        self._processing_thread.start()

    def _frame_accumulator(self, msg: ConsolePBMsg) -> None:
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
            time.sleep(0.01)

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
                        if self._on_msg_rx:
                            self._on_msg_rx(tracker.message())
                        uuid_delete_list.append(uuid)

                # Handle deleting any stale frames
                for uuid in uuid_delete_list:
                    self._in_progress_frames.pop(uuid)


class TransactionResponseObserver(MessageObserver):
    """ Observer for listening to the response of a specific message """

    def __init__(self, txn_uuid: int, timeout: Union[int, float]):
        # Register the observer to listen to all messages
        super().__init__(func=self._uuid_matcher_observer, msg_type=type(None), timeout=timeout)
        self._result: Optional[BasePBMsg] = None
        self._event = Event()
        self._timeout = timeout
        self._txn_uuid = txn_uuid

    def wait(self) -> Optional[BasePBMsg]:
        """
        Waits for the response to the message we're observing
        Returns:
            Response message
        """
        self._event.wait(timeout=self._timeout)
        return self._result

    def _uuid_matcher_observer(self, _msg: BasePBMsg) -> None:
        """
        Callback for the observer. Will only accept messages with the same UUID as
        the message we're waiting for.
        Args:
            _msg: Message being observed

        Returns:
            None
        """
        if not isinstance(_msg, BasePBMsg):
            return
        elif not self._event.is_set() and (self._txn_uuid == _msg.uuid):
            self._result = copy.copy(_msg)
            self._event.set()


class PredicateObserver(MessageObserver):
    """ Observer that accepts messages based on a user defined predicate """

    def __init__(self, func: Callable[[BasePBMsg], bool], qty: int = 1, timeout: Union[int, float] = 1.0):
        """
        Args:
            func: Predicate function that accepts a message and returns True if it should be accepted
            qty: Number of messages to accept before terminating
            timeout: How long to wait for the quantity limits to be satisfied
        """
        assert qty > 0, "Must observe at least one message"
        assert timeout > 0, "Timeout must be greater than zero"

        super().__init__(func=self._predicate_matcher_observer, msg_type=type(None), timeout=timeout)
        self._event = Event()
        self._msg_queue = Queue(qty)
        self._timeout = timeout
        self._predicate = func

    def wait(self) -> Optional[List[BasePBMsg]]:
        """
        Waits for the response(s) to the message predicate we're observing
        Returns:
            Response message(s)
        """
        self._event.wait(timeout=self._timeout)
        return list(self._msg_queue.queue)

    def _predicate_matcher_observer(self, msg: BasePBMsg) -> None:
        """
        Callback for the observer. Will only accept messages that pass the user defined
        predicate.
        Args:
            msg: Message being observed

        Returns:
            None
        """
        if not isinstance(msg, BasePBMsg):
            return

        # Accept the message if we can
        if not self._event.is_set() and not self._msg_queue.full() and self._predicate(msg):
            self._msg_queue.put(copy.copy(msg))

        # Notify the listener if we're done
        if self._msg_queue.full() and not self._event.is_set():
            self._event.set()
