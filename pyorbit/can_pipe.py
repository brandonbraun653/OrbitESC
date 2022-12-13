# **********************************************************************************************************************
#   FileName:
#       can_pipe.py
#
#   Description:
#       Core driver for interacting with an OrbitESC node over CAN bus
#
#   03/02/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import atexit
import can
import inspect
import re
import subprocess
import time
import uuid
import queue

from can import BusABC
from ifconfigparser import IfconfigParser
from loguru import logger
from typing import Any, Callable, Dict, List, Type, Union
from threading import Thread, RLock, Event
from pyorbit.can_messages import BaseMessage


class MessageObserver:
    """ Custom observer handle for reacting to a received CAN id """

    def __init__(self, func: Callable[[Union[can.Message, None]], None], arb_id: int, timeout: Union[float, int] = 0):
        """
        Args:
            func: Function to invoke upon receiving a message with the configured arbitration id
            arb_id: CAN arbitration ID to register against
            timeout: How long the observer should stay active (zero == Indefinite)
        """
        self.observer_function = func
        self.arbitration_id = arb_id
        self.timeout = timeout
        self.start_time = time.time()


class MessageQueue:
    """ Listener utility for holding CAN messages as they arrive """

    def __init__(self, msg: Any, qty: int = 0, timeout: Union[float, int] = 0, verbose: bool = False):
        """
        Args:
            msg: An object or class type derived from BaseMessage
            qty: How many messages to accumulate. If zero, accumulates infinite messages.
            timeout: How long to allow the accumulator to grab new messages. If zero, listen forever.
            verbose: Optionally log behavior
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
        self._arb_id = instantiated_msg.id()
        self._qty = qty
        self._timeout = timeout
        self._verbose = verbose
        self._msg_queue = queue.Queue(maxsize=qty)
        self._uuid = uuid.UUID(int=0)
        self._event = Event()
        self._filled = False
        self._observer_instance = MessageObserver(func=self._message_observer, arb_id=self._msg_type.id(),
                                                  timeout=self._timeout)

    def __del__(self):
        self.notify_expiry()
        time.sleep(0.1)
        logger.trace(f"Subscription {str(self._uuid)}: Terminated") if self._verbose else None

    @property
    def persistent(self) -> bool:
        """
        Returns:
            If the subscription never ends unless forcefully terminated
        """
        return self._qty == 0 and self._timeout == 0

    @property
    def expired(self) -> bool:
        """
        Checks to see if the queue's lifetime has expired
        Returns:
            True if expired, False if not
        """
        return self._timeout and ((time.time() - self._observer_instance.start_time) > self._timeout)

    @property
    def threshold_met(self) -> bool:
        """
        Returns:
            True if the queue is full, otherwise False
        """
        return self._msg_queue.full()

    @property
    def unique_id(self) -> uuid.UUID:
        return self._uuid

    @unique_id.setter
    def unique_id(self, val: uuid.UUID) -> None:
        self._uuid = val

    @property
    def arb_id(self):
        return self._arb_id

    @property
    def observer(self) -> MessageObserver:
        return self._observer_instance

    def get_messages(self, flush: bool = True) -> List[BaseMessage]:
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

    def notify_expiry(self) -> None:
        """
        External wakeup injection to notify that a message has expired. Allows an external
        thread of control to monitor the expiration state of this MessageQueue. The alternative
        would be spawning a thread per queue, which is a bit wasteful.

        Returns:
            None
        """
        self._event.set()

    def await_fulfillment_or_timeout(self) -> None:
        """
        Blocking function that only returns once the subscription's conditions are met.
            1. Subscription has timed out
            2. Subscription is looking for "qty" of packets and that qty exists
            3. Subscription is persistent, in which case will return immediately

        Returns:
            None
        """
        # Return immediately on any condition of the queue already meeting its termination requirements
        if self.persistent or self.expired or self.threshold_met:
            return

        # Block until something wakes this thread. Could be a timeout or queue fulfillment.
        # If the timeout occurs while the queue threshold isn't met, it relies on an external controller
        # to signal the timeout event, otherwise this would block forever.
        self._event.wait()
        self._event.clear()

    def _message_observer(self, msg: can.Message) -> None:
        """
        Observer to accept new messages and push them into the queue
        Args:
            msg: Message received

        Returns:
            None
        """
        # Handle timeout and bad input conditions
        if (msg.arbitration_id != self._msg_type.id()) or self.expired:
            return

        # Attempt to insert the new message into the queue
        try:
            msg_object = self._msg_type().unpack(msg)
            self._msg_queue.put(msg_object, block=False)

            # No limit? Notify on any message received. Otherwise notify when the limit is reached
            # or allow the timeout to expire.
            if self._qty == 0:
                self._event.set()
            elif self._msg_queue.full():
                self._event.set()

        except queue.Full:
            logger.trace(f"Subscription {str(self._uuid)}: Queue full") if self._verbose else None
            self._event.set()


class ObserverManager:

    def __init__(self):
        self._lock = RLock()
        self._dispatch_map = {}  # type: Dict[int, List[MessageObserver]]
        self._registry = {}  # type: Dict[uuid.UUID, MessageObserver]

    def add_observer(self, arb_id: int, observer: MessageObserver) -> uuid.UUID:
        """
        Adds a new observer of the given arbitration ID
        Args:
            arb_id: The CAN bus arbitration ID to register the observer against
            observer: The observer being registered

        Returns:
            Unique ID of the observer/arb_id combination as understood by the observer
        """
        with self._lock:
            # Initialize a new arbitration ID to register observers against
            if arb_id not in self._dispatch_map.keys():
                self._dispatch_map[arb_id] = []

            # Allocate the new observer entry
            new_id = uuid.uuid4()
            self._registry[new_id] = observer
            self._dispatch_map[arb_id].append(observer)

            return new_id

    def remove_observer(self, unique_id: uuid.UUID) -> None:
        """
        Removes an observer identified by a unique id
        Args:
            unique_id: The ID associated with the observer

        Returns:
            None
        """
        with self._lock:
            if unique_id not in self._registry.keys():
                return

            # Look up the tags used for registration
            observer = self._registry[unique_id]

            # Remove the registration
            self._dispatch_map[observer.arbitration_id].remove(observer)
            self._registry.pop(unique_id)

    def accept(self, message: Union[can.Message, None]) -> None:
        """
        Accepts a new CAN message to distribute to all registered observers
        Args:
            message: Observed message

        Returns:
            None
        """
        if not message:
            return

        with self._lock:
            try:
                for cb in self._dispatch_map[message.arbitration_id]:
                    cb.observer_function(message)
            except KeyError:
                pass  # No registered observers

    def prune(self) -> List[uuid.UUID]:
        """
        Removes timed out observers from active registration
        Returns:
            List of all IDs removed
        """
        removed_ids = []

        with self._lock:
            for key in list(self._registry.keys()):
                observer = self._registry[key]

                if observer.timeout and ((time.time() - observer.start_time) > observer.timeout):
                    logger.trace(f"Observer timeout of {observer.timeout} seconds for CAN arbitration id "
                                 f"{observer.arbitration_id}. Removing {str(key)}.")
                    del self._registry[key]
                    removed_ids.append(key)

        return removed_ids


class CANPipe:
    """ Interface to a number of ESC nodes communicating over CAN bus """

    def __init__(self, can_device: str, bit_rate: int):
        """
        Args:
            can_device: Device to communicate over as listed in ifconfig
            bit_rate: Effective CAN bus communication speed
        """
        # CAN related variables
        try:
            self._bus = can.ThreadSafeBus(bustype='socketcan', channel=can_device, bitrate=bit_rate)
        except OSError:
            logger.warning(f"Device {can_device} not found. Creating virtual CAN device.")
            self._bus = can.interface.Bus("virtual_" + can_device, bustype='virtual')

        self._default_reader = can.BufferedReader()
        self._message_notifier = can.Notifier(self._bus, [self._default_reader])

        # Internal thread related variables
        self._data_lock = RLock()
        self._kill_event = Event()
        self._reader_thread = Thread(target=self._can_message_handler, daemon=True, name="CANPipe_Reader")

        # Data management for subscriptions and observers
        self._observers = ObserverManager()
        self._subscriptions = {}  # type: Dict[uuid.UUID, MessageQueue]

        # Kick things off!
        atexit.register(self.shutdown)
        self._reader_thread.start()

    @classmethod
    def available_interfaces(cls) -> List[str]:
        """
        Returns:
            Available interfaces that match some kind of CAN bus driver
        """
        intf_list = IfconfigParser(subprocess.getoutput("ifconfig -a")).list_interfaces()
        return list(filter(re.compile(r"^can.*").match, intf_list))

    @property
    def bus(self) -> BusABC:
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
        # Kill the CAN bus message reader first to stop the influx of data. Allow some time to process
        # any straggling messages that may be left in the reader queue.
        self._message_notifier.stop()
        self._bus.shutdown()

        # Kill local resources last
        self._kill_event.set()
        self._reader_thread.join()

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
        q = MessageQueue(msg=msg, qty=qty, timeout=timeout)
        q.unique_id = self._observers.add_observer(arb_id=q.arb_id, observer=q.observer)

        with self._data_lock:
            self._subscriptions[q.unique_id] = q
            return q.unique_id

    def subscribe_observer(self, observer: MessageObserver) -> uuid.UUID:
        """
        Adds an observer into the OrbitESC runtime message parsing infrastructure

        Args:
            observer: The observer being added

        Returns:
            Unique identifier for the registered observer
        """
        return self._observers.add_observer(arb_id=observer.arbitration_id, observer=observer)

    def unsubscribe(self, unique_id: uuid.UUID) -> None:
        """
        Removes an observer or subscription from registration
        Args:
            unique_id: ID of the observer/subscription to remove

        Returns:
            None
        """
        self._observers.remove_observer(unique_id=unique_id)

        try:
            with self._data_lock:
                del self._subscriptions[unique_id]
        except KeyError:
            pass

    def get_subscription_data(self, unique_id: uuid.UUID, block: bool = True, flush: bool = True,
                              terminate: bool = False) -> List[BaseMessage]:
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
            # Prune any observers that have timed out
            removed = self._observers.prune()
            with self._data_lock:
                for key in removed:
                    try:
                        self._subscriptions[key].notify_expiry()
                    except KeyError:
                        continue

            # Process any new CAN messages
            msg = self._default_reader.get_message(timeout=0.001)
            self._observers.accept(msg)

            # Check for termination of the reader
            if self._default_reader.is_stopped:
                self._kill_event.set()

        logger.debug("Terminating OrbitESC internal CAN message dispatcher")
