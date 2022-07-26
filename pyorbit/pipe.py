# **********************************************************************************************************************
#   FileName:
#       pipe.py
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


class MessageObserver:
    """ Custom observer handle for reacting to a received CAN id """

    def __init__(self, func: Callable[[Union[can.Message, None], bool], None], arbitration_id: int,
                 timeout: Union[float, int, None] = None,
                 persistent: bool = False):
        """
        Args:
            func: Function to invoke upon receiving a message with the configured arbitration id
            arbitration_id: CAN arbitration ID to register against
            timeout: How long the observer should stay active (None == Indefinite)
            persistent: Should the observer stay registered once triggered?
        """
        self.observer_function = func
        self.arbitration_id = arbitration_id
        self.timeout = timeout
        self.persistent = persistent
        self.start_time = time.time()


class CANPipe:
    """ Interface to a number of ESC nodes communicating over CAN bus """

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
        self._rx_observers = {"active_ids": []}
        self._periodic_messages = []

        # Kick things off!
        self._reader_thread.start()
        atexit.register(self.shutdown)

    @classmethod
    def available_interfaces(cls) -> List[str]:
        """
        Returns:
            Available interfaces that match some kind of CAN bus driver
        """
        intf_list = IfconfigParser(subprocess.getoutput("ifconfig -a")).list_interfaces()
        return list(filter(re.compile(r"^can.*").match, intf_list))

    @property
    def bus(self) -> can.ThreadSafeBus:
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
        self._kill_event.set()
        self._reader_thread.join()
        self._default_reader.stop()

    def subscribe(self, observer: MessageObserver) -> uuid.UUID:
        """
        Adds an observer into the OrbitESC runtime message parsing infrastructure

        Args:
            observer: The observer being added

        Returns:
            Unique identifier for the registered observer
        """
        with self._data_lock:
            # Register the observer against the UUID
            new_uuid = uuid.uuid4()
            self._rx_observers[new_uuid] = observer

            # Add to the arbitration id observer list
            if observer.arbitration_id not in self._rx_observers.keys():
                self._rx_observers[observer.arbitration_id] = []
                self._rx_observers["active_ids"].append(observer.arbitration_id)

            self._rx_observers[observer.arbitration_id].append(observer)
            observer.start_time = time.time()

        return new_uuid

    def unsubscribe(self, unique_id: uuid.UUID) -> None:
        """
        Removes an observer from registration
        Args:
            unique_id: ID of the observer to remove

        Returns:
            None
        """
        with self._data_lock:
            if unique_id in self._rx_observers.keys():
                # Remove the observer as registered with the unique ID
                observer = self._rx_observers[unique_id]   # type: MessageObserver
                self._rx_observers.pop(unique_id, None)

                # Remove the observer from the CAN arbitration ID list
                try:
                    self._rx_observers[observer.arbitration_id].remove(observer)
                    if not self._rx_observers[observer.arbitration_id]:
                        self._rx_observers["active_ids"].remove(observer.arbitration_id)
                except ValueError:
                    pass

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

        def notifier_observer(can_msg: can.Message, timeout: bool):
            nonlocal success, node_id

            if can_msg:
                ping_msg = Ping().unpack(bytes(can_msg.data))
                success = (ping_msg.src.node_id == node_id and not timeout)

            wake_up.set()

        # Register the observer for the ping message response
        observer = MessageObserver(func=notifier_observer, arbitration_id=Ping.id(), timeout=0.5)
        self.subscribe(observer)

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
        """
        Checks active observer registrations for any timeouts. Prunes the registration list.
        Returns:
            None
        """
        with self._data_lock:
            for active_id in self._rx_observers["active_ids"]:
                to_remove = []

                # Check the timeouts
                for observer in self._rx_observers[active_id]:  # type: MessageObserver
                    if observer.timeout and (time.time() - observer.start_time > observer.timeout):
                        observer.observer_function(None, True)

                        # Mark for removal if not a persistent registration
                        if not observer.persistent:
                            to_remove.append(observer)

                # Remove all timed-out non-persistent observers
                for item in to_remove:
                    self._rx_observers[active_id].remove(item)

    def _process_rx_messages(self) -> None:
        """
        Receives new CAN messages and then dispatches the data to registered observers
        Returns:
            None
        """
        with self._data_lock:
            while msg := self._default_reader.get_message(timeout=0.0):     # type: Union[can.Message, None]
                # Skip if not registered
                if msg.arbitration_id not in self._rx_observers.keys():
                    continue

                # Invoke all registered observers for this message
                for cb in self._rx_observers[msg.arbitration_id]:
                    cb.observer_function(can_msg=msg, timeout=False)
