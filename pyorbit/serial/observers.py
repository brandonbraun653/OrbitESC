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

import time
from functools import cmp_to_key
from loguru import logger
from typing import Dict, List, Callable, Optional
from pyorbit.serial.parameters import ParameterType
from pyorbit.serial.pipe import SerialPipeObserver
from pyorbit.serial.messages import *
from pyorbit.observer import MessageObserver
from threading import Thread, Lock


class ConsoleObserver(MessageObserver):
    """ Simple observer for listening to console messages off the OrbitESC debug port """

    class FrameBuffer:
        def __init__(self):
            self.start_time = time.time()
            self.total_frames = 0
            self.frames = []  # type: List[ConsoleMessage]

        def message(self) -> str:
            self.frames = sorted(self.frames, key=cmp_to_key(lambda x1, x2: x1.frame_number - x2.frame_number))
            final_msg = ""
            for frame in self.frames:
                final_msg = final_msg + frame.data.decode("utf-8")
            return final_msg

    def __init__(self, on_msg_rx: Callable[[str], None]):
        super().__init__(func=self._frame_accumulator, msg_type=ConsoleMessage)
        self._frame_lock = Lock()
        self._on_msg_rx = on_msg_rx
        self._in_progress_frames = {}  # type: Dict[int, ConsoleObserver.FrameBuffer]
        self._processing_thread = Thread(target=self._frame_processor, name="FrameProcessor", daemon=True)
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


class ParameterObserver(MessageObserver):
    """ Simple observer for interacting with the parameter registry off the OrbitESC debug port """

    def __init__(self, pipe: SerialPipeObserver):
        super().__init__(func=self._observer_func, msg_type=ParamIOMessage)
        self._com_pipe = pipe

    def get(self, param: ParameterId) -> Optional[ParameterType]:
        """
        Requests the current value of a parameter
        Args:
            param: Which parameter to get

        Returns:
            Deserialized value
        """
        # Format and publish the request
        msg = ParamIOMessage()
        msg.sub_id = MessageSubId.ParamIO_Get
        msg.param_id = param

        # Push the request onto the wire
        logger.trace(f"Requesting parameter {repr(param)}")
        start_time = time.time()
        rsp = self._com_pipe.write_and_wait(msg, 3.0)

        # Server denied the request for some reason
        if isinstance(rsp, AckNackMessage):
            logger.error(f"GET parameter {repr(param)} failed with status code: "
                         f"{repr(StatusCode(rsp.status_code))}")
            return None

        # Deserialize the data returned
        if isinstance(rsp, ParamIOMessage):
            logger.trace(f"Transaction completed in {time.time() - start_time:.2f} seconds for {repr(param)}")
            try:
                if rsp.param_type == ParameterEncoding.STRING:
                    return rsp.data.decode('utf-8')
                elif rsp.param_type == ParameterEncoding.FLOAT or rsp.param_type == ParameterEncoding.DOUBLE:
                    return float(struct.unpack("<f", rsp.data)[0])
                elif rsp.param_type == ParameterEncoding.UINT8 or rsp.param_type == ParameterEncoding.UINT16 or \
                        rsp.param_type == ParameterEncoding.UINT32:
                    return int.from_bytes(rsp.data, byteorder='little')
                elif rsp.param_type == ParameterEncoding.BOOL:
                    assert len(rsp.data) == 1, f"Expected length 1 for bool type but got {len(rsp.data)}"
                    assert rsp.data[0] == 0 or rsp.data[0] == 1, f"Expected 0 or 1 for bool type but got {rsp.data[0]}"
                    return bool(rsp.data[0])
                else:
                    logger.warning(f"Don't know how to decode parameter type {rsp.param_type}")
                    return rsp.data
            except ValueError:
                logger.error(f"Failed to decode response: {msg}")
                return None

        logger.error("No response from server")
        return None

    def set(self, param: ParameterId, value: ParameterType) -> bool:
        """
        Sets the value of a parameter
        Args:
            param: Which parameter to set
            value: Value being assigned

        Returns:
            True if the operation succeeds, False if not
        """
        from pyorbit.app.parameters.util import parameter_encoding

        # Build the base of the message
        msg = ParamIOMessage()
        msg.sub_id = MessageSubId.ParamIO_Set
        msg.param_id = param
        msg.param_type = parameter_encoding(param)

        # Serialize the data to be sent
        if msg.param_type == ParameterEncoding.STRING:
            msg.data = str(value).encode('utf-8')
        elif msg.param_type == ParameterEncoding.FLOAT:
            msg.data = struct.pack('<f', float(value))
        elif msg.param_type == ParameterEncoding.DOUBLE:
            msg.data = struct.pack('<d', float(value))
        elif msg.param_type == ParameterEncoding.UINT8 or msg.param_type == ParameterEncoding.BOOL:
            msg.data = struct.pack('<B', int(value))
        elif msg.param_type == ParameterEncoding.UINT16:
            msg.data = struct.pack('<H', int(value))
        elif msg.param_type == ParameterEncoding.UINT32:
            msg.data = struct.pack('<I', int(value))
        else:
            logger.error(f"Don't know how to serialize parameter type {msg.param_type}")
            return False

        # Push the request onto the wire
        rsp = self._com_pipe.write_and_wait(msg, 3.0)

        # Parse the result
        if not rsp:
            logger.error("No valid response from server")
            return False

        assert isinstance(rsp, AckNackMessage), f"Expected AckNackMessage but got {type(rsp)}"
        if not rsp.ack:
            logger.error(f"SET parameter {repr(param)} failed with status code: {repr(StatusCode(rsp.status_code))}")
            return False
        else:
            return True

    def load(self) -> bool:
        """
        Requests a complete reload from disk of all supported parameters
        Returns:
            True if the operation succeeds, False if not
        """
        sub_id = self._com_pipe.subscribe(AckNackMessage, qty=1, timeout=5.0)

        msg = ParamIOMessage()
        msg.sub_id = MessageSubId.ParamIO_Load

        self._com_pipe.write(msg.serialize())
        messages = self._com_pipe.get_subscription_data(sub_id, terminate=True)
        for rsp in messages:  # type: AckNackMessage
            if rsp.uuid != msg.uuid:
                continue
            else:
                return rsp.ack

    def store(self) -> bool:
        """
        Requests a complete serialize to disk of all supported parameters
        Returns:
            True if the operation succeeds, False if not
        """
        sub_id = self._com_pipe.subscribe(AckNackMessage, qty=1, timeout=5.0)

        msg = ParamIOMessage()
        msg.sub_id = MessageSubId.ParamIO_Sync

        self._com_pipe.write(msg.serialize())
        messages = self._com_pipe.get_subscription_data(sub_id, terminate=True)
        for rsp in messages:  # type: AckNackMessage
            if rsp.uuid != msg.uuid:
                continue
            else:
                return rsp.ack

    def _observer_func(self, msg: ParamIOMessage) -> None:
        """ Stub function required for integration with the MessageObserver class """
        pass
