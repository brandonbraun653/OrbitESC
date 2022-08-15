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
import copy
import inspect
import re
import sched
import subprocess
import time
import uuid
import queue
from ifconfigparser import IfconfigParser
from loguru import logger
from typing import Any, Callable, Dict, List, Type, Union
from threading import Thread, RLock, Event, Condition
from pyorbit.messages import BaseMessage


class MessageObserver:
    """ Custom observer handle for reacting to a received CAN id """

    def __init__(self, func: Callable[[Union[can.Message, None], bool], None], arb_id: int,
                 timeout: Union[float, int, None] = None,
                 persistent: bool = False):
        """
        Args:
            func: Function to invoke upon receiving a message with the configured arbitration id
            arb_id: CAN arbitration ID to register against
            timeout: How long the observer should stay active (None == Indefinite)
            persistent: Should the observer stay registered once triggered?
        """
        self.observer_function = func
        self.arbitration_id = arb_id
        self.timeout = timeout
        self.persistent = persistent
        self.start_time = time.time()


class MessageQueue:
    """ Listener utility for holding CAN messages as they arrive """

    def __init__(self, msg: Any, qty: int = 0, timeout: Union[float, int, None] = None):
        """
        Args:
            msg: An object or class type derived from BaseMessage
            qty: How many messages to accumulate. If zero, accumulates infinite messages.
            timeout: How long to allow the accumulator to grab new messages. If None, listen forever.
        """
        # Sanity check the input message prototype. Assumes that the type will be trivially constructible
        # in the case a class type is passed in.
        if inspect.isclass(msg):
            instantiated_msg = msg()
        else:
            instantiated_msg = msg

        if not isinstance(instantiated_msg, BaseMessage):
            raise ValueError(f"Class prototype [{type(instantiated_msg).__name__}] must be derived from BaseMessage")

        # All good. Initialize the rest of the class.
        self._msg_type = instantiated_msg.__class__  # type: Type[BaseMessage]
        self._qty = qty
        self._timeout = timeout
        self._msg_queue = queue.Queue(maxsize=qty)  # type: "Queue[BaseMessage]"
        self._uuid = uuid.uuid4()
        self._data_ready_event = Event()
        self._observer_instance = MessageObserver(func=self._message_observer, arb_id=self._msg_type.id(),
                                                  timeout=self._timeout, persistent=True)

    @property
    def expired(self) -> bool:
        """
        Checks to see if the queue's lifetime has expired
        Returns:
            True if expired, False if not
        """
        if self._timeout and ((time.time() - self._observer_instance.start_time) > self._timeout):
            # Unblock any thread waiting on timeouts
            self._data_ready_event.set()
            return True
        else:
            return False

    @property
    def unique_id(self) -> uuid.UUID:
        return self._uuid

    @property
    def observer(self) -> MessageObserver:
        return self._observer_instance

    def await_fulfillment_or_timeout(self) -> None:
        """
        Blocks current thread until notification is sent
        Returns:
            None
        """
        self._data_ready_event.wait()
        self._data_ready_event.clear()

    def get_messages(self, flush: bool = True) -> List[can.Message]:
        """
        Retrieves all messages in the subscription queue
        Args:
            flush: If true, empties the queue of data

        Returns:
            All available CAN messages
        """
        if self._msg_queue.empty():
            return []

        if flush:
            return [self._msg_queue.get() for _ in range(self._msg_queue.qsize())]
        else:
            return list(self._msg_queue)

    def _message_observer(self, msg: can.Message, timeout: bool) -> None:
        """
        Observer to accept new messages and push them into the queue
        Args:
            msg: Message received
            timeout: Whether the message timed out

        Returns:
            None
        """
        # Handle timeout and bad input conditions
        if timeout or not msg:
            return

        # Don't listen to the wrong message or if timeout has elapsed
        if (msg.arbitration_id != self._msg_type.id()) or self.expired:
            return

        try:
            # Guarantee message insertion if it exists
            msg_object = self._msg_type().unpack(msg)
            self._msg_queue.put(msg_object, block=False)

            # Check fulfillment of quantity requirements
            if (self._qty == 0) or (self._msg_queue.qsize() >= self._qty):
                self._data_ready_event.set()
        except queue.Full:
            logger.trace(f"Message queue full: {repr(self._uuid)}")


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

        # Subscription data
        self._subscriptions = {}  # type: Dict[uuid.UUID, MessageQueue]

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

    def subscribe(self, msg: Any, qty: int = 0, timeout: Union[int, float, None] = None) -> uuid.UUID:
        """
        Creates a subscription to the given CAN message type. In the background, the subscription
        will accumulate messages in a queue for later retrieval.

        Args:
            msg: An object or class type derived from BaseMessage
            qty: How many messages to receive. If zero, accept infinite messages.
            timeout: How long in seconds to wait for "qty" of messages to arrive

        Returns:
            Unique ID of the subscription
        """
        with self._data_lock:
            q = MessageQueue(msg=msg, qty=qty, timeout=timeout)
            self._subscriptions[q.unique_id] = q
            self._add_new_observer(unique_id=q.unique_id, observer=q.observer)

        return q.unique_id

    def subscribe_observer(self, observer: MessageObserver) -> uuid.UUID:
        """
        Adds an observer into the OrbitESC runtime message parsing infrastructure

        Args:
            observer: The observer being added

        Returns:
            Unique identifier for the registered observer
        """
        new_uuid = uuid.uuid4()
        self._add_new_observer(unique_id=new_uuid, observer=observer)

        return new_uuid

    def unsubscribe(self, unique_id: uuid.UUID) -> None:
        """
        Removes an observer or subscription from registration
        Args:
            unique_id: ID of the observer/subscription to remove

        Returns:
            None
        """
        with self._data_lock:
            if unique_id in self._rx_observers.keys():
                # Remove the observer as registered with the unique ID
                observer = self._rx_observers[unique_id]  # type: MessageObserver
                self._rx_observers.pop(unique_id, None)

                # Remove the observer from the CAN arbitration ID list
                try:
                    self._rx_observers[observer.arbitration_id].remove(observer)
                    if not self._rx_observers[observer.arbitration_id]:
                        self._rx_observers["active_ids"].remove(observer.arbitration_id)
                except ValueError:
                    pass

            if unique_id in self._subscriptions.keys():
                self._subscriptions.pop(unique_id)

    def get_subscription_data(self, unique_id: uuid.UUID, block: bool = True, flush: bool = True,
                              terminate: bool = False) -> List[can.Message]:
        """
        Gets all data associated with a subscription
        Args:
            unique_id: ID returned from the subscription
            block: Wait for the subscription's data quantity to arrive or timeout to expire
            flush: Optionally flush the data from the subscription queue
            terminate: Unsubscribe from the associated CAN message

        Returns:
            List of messages enqueued for the subscription
        """
        try:
            # Wait for the data to arrive or timeout to occur?
            if block:
                self._subscriptions[unique_id].await_fulfillment_or_timeout()

            # Grab the data. Might not be anything there.
            sub_data = self._subscriptions[unique_id].get_messages(flush=flush)

            # Unregister the subscription with the observer infrastructure? Renders the unique_id useless.
            if terminate:
                self.unsubscribe(unique_id)

            return sub_data
        except KeyError as e:
            logger.error(f"{repr(e)}")

        # Default return value
        return []

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
            while msg := self._default_reader.get_message(timeout=0.0):  # type: Union[can.Message, None]
                # Skip if not registered
                if msg.arbitration_id not in self._rx_observers.keys():
                    continue

                # Invoke all registered observers for this message
                for cb in self._rx_observers[msg.arbitration_id]:
                    cb.observer_function(msg, False)

    def _add_new_observer(self, unique_id: uuid.UUID, observer: MessageObserver) -> None:
        """
        Adds a new observer to the RX listeners

        Args:
            unique_id: Unique ID of the observer
            observer: Observer object being registered

        Returns:
            None
        """
        with self._data_lock:
            # Map the UniqueID <-> Observer Object
            self._rx_observers[unique_id] = observer

            # Register the arbitration IDs to the actively listened to ID list
            if observer.arbitration_id not in self._rx_observers.keys():
                self._rx_observers[observer.arbitration_id] = []
                self._rx_observers["active_ids"].append(observer.arbitration_id)

            # Link the observer to the arbitration ID
            self._rx_observers[observer.arbitration_id].append(observer)
            observer.start_time = time.time()


if __name__ == "__main__":
    from pyorbit.messages import SystemID

    MessageQueue(msg=SystemID)
    MessageQueue(msg=SystemID())
