# **********************************************************************************************************************
#   FileName:
#       esc.py
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
import time
import uuid
from ifconfigparser import IfconfigParser
from loguru import logger
from typing import Callable, List, Union
from pyorbit.messages import NodeID, Ping
from threading import Thread, RLock, Event


# SubProcessCommand(cmd="sudo modprobe can").execute()
# SubProcessCommand(cmd="sudo modprobe can-raw").execute()
# SubProcessCommand(cmd=f"sudo ip link set can0 type can bitrate {can_bitrate}")
# SubProcessCommand(cmd=f"sudo ip link set can0 up type can bitrate {can_bitrate}").execute()

class OrbitMessageCallback:
    """ Custom callback handle for execution on a received CAN id """

    def __init__(self, func: Callable[[Union[can.Message, None], bool], None], arbitration_id: int,
                 timeout: Union[float, int, None] = None,
                 persistent: bool = False):
        """
        Args:
            func: Function to invoke
            arbitration_id: CAN arbitration ID to register against
            timeout: How long the callback should stay active (None == Indefinite)
            persistent: Should the callback stay registered once triggered?
        """
        self.callback_function = func
        self.arbitration_id = arbitration_id
        self.timeout = timeout
        self.persistent = persistent
        self.start_time = time.time()


class OrbitESC:
    """ Interface to an ESC node communicating over CAN bus """

    def __init__(self, can_device: str, bit_rate: int):
        """
        Args:
            can_device: Device to communicate over as listed in ifconfig
            bit_rate: Effective CAN bus communication speed
        """
        # CAN related variables
        self._bus = can.ThreadSafeBus(bustype='socketcan', channel=can_device, bitrate=bit_rate)
        self._default_reader = can.BufferedReader()
        self._message_notifier = can.Notifier(self._bus, [self._default_reader])

        # Internal thread related variables
        self._data_lock = RLock()
        self._kill_event = Event()
        self._reader_thread = Thread(target=self._can_message_handler)
        self._rx_message_callbacks = {"active_ids": []}
        self._periodic_messages = []

        # Kick things off!
        self._reader_thread.start()
        atexit.register(self.shutdown)

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self._kill_event.set()
        self._reader_thread.join()
        self._default_reader.stop()

    def _can_message_handler(self) -> None:
        """
        The OrbitESC internal thread to handle async message responses or other kinds of message
        based actions that need to occur under the hood.

        Returns:
            None
        """
        logger.debug("Starting OrbitESC internal CAN message dispatcher")

        while not self._kill_event.is_set():
            self._poll_pending_rx_timeouts()
            self._process_rx_messages()

            # Check for termination of the reader
            if self._default_reader.is_stopped:
                self._kill_event.set()

        logger.debug("Terminating OrbitESC internal CAN message dispatcher")

    def _poll_pending_rx_timeouts(self) -> None:
        with self._data_lock:
            for active_id in self._rx_message_callbacks["active_ids"]:
                to_remove = []

                # Check the timeouts
                for callback in self._rx_message_callbacks[active_id]:  # type: OrbitMessageCallback
                    if time.time() - callback.start_time > callback.timeout:
                        callback.callback_function(None, True)

                        # Mark for removal if not a persistent registration
                        if not callback.persistent:
                            to_remove.append(callback)

                # Remove all timed-out non-persistent callbacks
                for item in to_remove:
                    self._rx_message_callbacks[active_id].remove(item)

    def _process_rx_messages(self) -> None:
        with self._data_lock:
            while msg := self._default_reader.get_message(timeout=0.0):     # type: Union[can.Message, None]
                if msg.arbitration_id not in self._rx_message_callbacks.keys():
                    continue

                for cb in self._rx_message_callbacks[msg.arbitration_id]:
                    cb.callback_function(can_msg=msg, timeout=False)

    @classmethod
    def available_interfaces(cls) -> List[str]:
        intf_list = IfconfigParser(subprocess.getoutput("ifconfig -a")).list_interfaces()
        return list(filter(re.compile(r"^can.*").match, intf_list))

    @property
    def bus(self) -> can.ThreadSafeBus:
        """
        Returns:
            Instance of the raw CAN bus connection
        """
        return self._bus

    def register_callback(self, callback: OrbitMessageCallback) -> uuid.UUID:
        """
        Adds a callback into the OrbitESC runtime message parsing infrastructure

        Args:
            callback: The callback being added

        Returns:
            Unique identifier for the registered callback
        """
        with self._data_lock:
            # Register the callback against the UUID
            new_uuid = uuid.uuid4()
            self._rx_message_callbacks[new_uuid] = callback

            # Add to the arbitration id callback list
            if callback.arbitration_id not in self._rx_message_callbacks.keys():
                self._rx_message_callbacks[callback.arbitration_id] = []
                self._rx_message_callbacks["active_ids"].append(callback.arbitration_id)

            self._rx_message_callbacks[callback.arbitration_id].append(callback)
            callback.start_time = time.time()

        return new_uuid

    def remove_callback(self, unique_id: uuid.UUID) -> None:
        """
        Removes a callback from registration
        Args:
            unique_id: ID of the callback to remove

        Returns:
            None
        """
        with self._data_lock:
            if unique_id in self._rx_message_callbacks.keys():
                # Remove the callback as registered with the unique ID
                callback = self._rx_message_callbacks[unique_id]   # type: OrbitMessageCallbac
                self._rx_message_callbacks.pop(unique_id, None)

                # Remove the callback from the CAN arbitration ID list
                try:
                    self._rx_message_callbacks[callback.arbitration_id].remove(callback)
                    if not self._rx_message_callbacks[callback.arbitration_id]:
                        self._rx_message_callbacks["active_ids"].remove(callback.arbitration_id)
                except ValueError:
                    pass

    def connect(self, node_id: NodeID) -> bool:
        if self.bus.state != can.bus.BusState.ACTIVE:
            logger.error(f"Failed to initialize CAN bus. State is {self._bus.state}.")
            return False
        else:
            return self.ping(node_id)

    def ping(self, node_id: NodeID) -> bool:
        """
        Communicates with the desired to node to see if it's alive
        Args:
            node_id: Which node to talk with

        Returns:
            True if the node responds, False if not
        """
        success = False
        wake_up = Event()

        def notifier_callback(can_msg: can.Message, timeout: bool):
            nonlocal success, node_id

            if can_msg:
                ping_msg = Ping().unpack(bytes(can_msg.data))
                success = (ping_msg.src.node_id == node_id and not timeout)

            wake_up.set()

        # Register the callback for the ping message response
        callback = OrbitMessageCallback(func=notifier_callback, arbitration_id=Ping.id(), timeout=0.5)
        self.register_callback(callback)

        # Send the ping to the destination node
        ping = Ping()
        ping.dst.node_id = node_id
        ping.src.node_id = NodeID.NODE_PC
        msg = can.Message(arbitration_id=ping.id(), data=ping.pack())
        logger.debug(f"Sending ping to node {node_id}")
        self.bus.send(msg)

        # Wait for the message to arrive or timeout
        wake_up.wait(timeout=None)
        logger.debug(f"Ping {'received' if success else 'not received'} from node {node_id}")
        return success
