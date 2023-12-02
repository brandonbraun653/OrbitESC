# **********************************************************************************************************************
#   FileName:
#       client.py
#
#   Description:
#       Client to connect with an OrbitESC device over serial
#
#   01/20/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from __future__ import annotations

import atexit
import time
from functools import wraps
from loguru import logger
from typing import Any, Callable

from pyorbit.serial.intf.system_control import StreamPhaseCurrentsMessage, SystemResetMessage
from pyorbit.serial.pipe import SerialPipePublisher
from pyorbit.exceptions import NotOnlineException
from pyorbit.serial.messages import *
from pyorbit.observers import MessageObserver
from pyorbit.serial.observers import ConsoleObserver
from pyorbit.serial.parameters import SetActivityLedBlinkScalerMessage, ParameterId
from pyorbit.serial.controllers import ParameterController
from threading import Event, Thread


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
    def _impl(self: OrbitClient, *method_args, **method_kwargs):
        # Ensure this decorator is only used on OrbitESC classes
        assert isinstance(self, OrbitClient)
        if not self.is_online:
            raise NotOnlineException()

        # Execute the function as normal
        method_output = method(self, *method_args, **method_kwargs)
        return method_output

    return _impl


class OrbitClient:
    """ High level serial client to connect with the debug server running on OrbitESC """

    def __init__(self, port: str, baudrate: int):
        """
        Args:
            port: Serial port endpoint to connect with
            baudrate: Desired communication baudrate
        """
        self._transport = SerialPipePublisher()
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
        self._param_observer = ParameterController(pipe=self.com_pipe)
        self.com_pipe.subscribe_observer(MessageObserver(func=self._observer_esc_tick, msg_type=SystemTickMessage))
        self.com_pipe.subscribe_observer(ConsoleObserver(on_msg_rx=lambda x: logger.info(x.strip('\n'))))

        atexit.register(self._teardown)

    @property
    def com_pipe(self) -> SerialPipePublisher:
        return self._transport

    @property
    def is_online(self) -> bool:
        return self._online

    @property
    def parameter(self) -> ParameterController:
        return self._param_observer

    def ping(self) -> bool:
        """
        Returns:
            True if the device was ping-able, False otherwise
        """
        sub_id = self.com_pipe.subscribe(msg=PingMessage, qty=1, timeout=5.0)
        self.com_pipe.write(PingMessage().serialize())
        responses = self.com_pipe.get_subscription_data(sub_id, terminate=True)
        if not responses:
            logger.warning("Node did not respond to ping")
        return bool(responses)

    def stream_phase_currents(self, enable: bool) -> bool:
        """
        Enable or disable the streaming of phase current data from the ESC
        Args:
            enable: True to enable, False to disable

        Returns:
            True if the command was successful, False otherwise
        """
        rsp = self.com_pipe.write_and_wait(StreamPhaseCurrentsMessage(enable=enable), timeout=3.0)
        return self.com_pipe.process_ack_nack_response(rsp, f"Failed to {'enable' if enable else 'disable'} "
                                                            f"phase current streaming")

    def system_reset(self) -> None:
        """
        Requests a full system reset from the ESC
        Returns:
            None
        """
        self.com_pipe.write(SystemResetMessage().serialize())


    def set_activity_led_blink_scaler(self, scaler: float = 1.0) -> bool:
        """
        Sets the activity LED blink scaler
        Args:
            scaler: New scaling constant to speed up or slow down the blink rate

        Returns:
            True if the command was successful, False otherwise

        Notes:
            The default scaler is 1.0, which means the LED will blink at the default pre-programmed rate.
        """
        sub_id = self.com_pipe.subscribe(msg=AckNackMessage, qty=1, timeout=5.0)
        self.com_pipe.write(SetActivityLedBlinkScalerMessage(scaler=scaler).serialize())
        responses = self.com_pipe.get_subscription_data(sub_id, terminate=True)
        if not responses:
            logger.warning("Node did not respond to set activity LED blink scaler command")
            return False
        else:
            return responses[0].ack

    def put(self, data: bytes) -> None:
        """
        Enqueues a new message for transmission
        Args:
            data: Byte data to transmit

        Returns:
            None
        """
        self.com_pipe.write(data)

    def close(self) -> None:
        return self._teardown()

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
            if self._online and ((time.time() - self._time_last_online) > 3.0):
                logger.warning("Serial link is offline")
                self._online = False

    def _observer_esc_tick(self, msg: SystemTickMessage) -> None:
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
