import inspect
import queue
import time
import uuid
from threading import Event
from typing import Callable, Union, Any, List
from loguru import logger


class ObserverUniqueId:
    """ Unique ID for an observer type """

    def __init__(self):
        self._uuid = uuid.UUID(int=0)

    @property
    def unique_id(self) -> uuid.UUID:
        return self._uuid

    @unique_id.setter
    def unique_id(self, val: uuid.UUID) -> None:
        self._uuid = val


class MessageObserver(ObserverUniqueId):
    """ Custom observer handle for reacting to a received message """

    def __init__(self, func: Callable[[Union[Any, None]], None], msg_type: Any, timeout: Union[float, int] = 0):
        """
        Args:
            func: Function to invoke upon receiving a message with the configured arbitration id
            msg_type: Message type to register against. Use None to accept all messages.
            timeout: How long the observer should stay active (zero == Indefinite)
        """
        super().__init__()
        self.observer_function = func
        self.msg_type = msg_type if isinstance(msg_type, type) else type(msg_type)
        self.timeout = timeout
        self.start_time = time.time()


class MessageQueue(ObserverUniqueId):
    """ Listener utility for holding messages as they arrive """

    def __init__(self, msg: Any, qty: int = 0, timeout: Union[float, int] = 0, verbose: bool = False):
        """
        Args:
            msg: An object or class type for the desired message type to accumulate
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

        # All good. Initialize the rest of the class.
        super().__init__()
        self._msg_type = instantiated_msg.__class__
        self._qty = qty
        self._timeout = timeout
        self._verbose = verbose
        self._msg_queue = queue.Queue(maxsize=qty)
        self._event = Event()
        self._filled = False
        self._observer_instance = MessageObserver(func=self._message_observer, msg_type=self._msg_type,
                                                  timeout=self._timeout)

    def __del__(self):
        self.notify_expiry()
        time.sleep(0.1)
        logger.trace(f"Subscription {str(self.unique_id)}: Terminated") if self._verbose else None

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
    def listen_to_type(self):
        return self._msg_type

    @property
    def observer(self) -> MessageObserver:
        return self._observer_instance

    def get_messages(self, flush: bool = True) -> List[Any]:
        """
        Retrieves all messages in the subscription queue
        Args:
            flush: If true, empties the queue of data

        Returns:
            All available messages
        """
        if self._msg_queue.empty():
            return []

        if flush:
            return [self._msg_queue.get() for _ in range(self._msg_queue.qsize())]
        else:
            return list(self._msg_queue.queue)

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

    def _message_observer(self, msg: Any) -> None:
        """
        Observer to accept new messages and push them into the queue
        Args:
            msg: Message received

        Returns:
            None
        """
        # Handle timeout and bad input conditions
        if not isinstance(msg, self._msg_type) or self.expired:
            return

        # Attempt to insert the new message into the queue
        try:
            self._msg_queue.put(msg, block=False)

            # No limit? Notify on any message received. Otherwise, notify when the limit is reached
            # or allow the timeout to expire.
            if self._qty == 0:
                self._event.set()
            elif self._msg_queue.full():
                self._event.set()

        except queue.Full:
            logger.trace(f"Subscription {str(self._uuid)}: Queue full") if self._verbose else None
            self._event.set()
