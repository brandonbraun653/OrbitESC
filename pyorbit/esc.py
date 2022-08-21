# **********************************************************************************************************************
#   FileName:
#       esc.py
#
#   Description:
#       Interface to control an ESC
#
#   07/24/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from __future__ import annotations

import atexit
import copy

import can
import time
import uuid
from enum import IntEnum
from loguru import logger
from functools import wraps
from threading import Event, Thread
from typing import Any, Callable, List, Union
from pyorbit.pipe import CANPipe, MessageObserver
from pyorbit.messages import NodeID, Ping, SystemTick, SystemMode, SetSystemMode, EmergencyHalt, SetMotorSpeed, \
    SystemReset
from pyorbit.exceptions import NotOnlineException


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
    def _impl(self: OrbitESC, *method_args, **method_kwargs):
        # Ensure this decorator is only used on OrbitESC classes
        assert isinstance(self, OrbitESC)
        if not self.is_online:
            raise NotOnlineException()

        # Execute the function as normal
        method_output = method(self, *method_args, **method_kwargs)
        return method_output

    return _impl


class OrbitESC:
    """ Core class for interacting/controlling an ESC over CAN bus """

    class Mode(IntEnum):
        """ Mimics the ModeId_t enumeration in sys_mode_base.hpp """
        Idle = 0
        Armed = 1
        Park = 2
        Ramp = 3
        Run = 4
        Fault = 5

        def __str__(self):
            mapping = {OrbitESC.Mode.Idle.value: "Idle",
                       OrbitESC.Mode.Armed.value: "Armed",
                       OrbitESC.Mode.Park.value: "Park",
                       OrbitESC.Mode.Ramp.value: "Ramp",
                       OrbitESC.Mode.Run.value: "Run",
                       OrbitESC.Mode.Fault.value: "Fault"}

            try:
                return mapping[self.value]
            except KeyError:
                return "Unknown"

        @classmethod
        def switchable_modes(cls) -> List[OrbitESC.Mode]:
            """
            Returns:
                List of modes that can be directly switched into
            """
            return [cls.Idle, cls.Armed, cls.Run, cls.Fault]

    def __init__(self, dst_node: NodeID, this_node: NodeID = NodeID.NODE_PC, device: str = "can0",
                 bit_rate: int = 1000000):
        """
        Args:
            this_node: Node to communicate as
            dst_node: Which node to communicate with
            device: Host PC CAN interface to use for communication
        """
        self._dst_node = dst_node
        self._src_node = this_node
        self._data_pipe = CANPipe(can_device=device, bit_rate=bit_rate)
        self._online = False
        self._time_last_online = 0
        self._last_tick = 0

        # Start up the background thread
        self._kill_signal = Event()
        self._notify_signal = Event()
        self._thread = Thread(target=self._background_thread, daemon=True,
                              name=f"OrbitESC_BackgroundThread_Node{dst_node}")
        self._thread.start()

        # Register known observers
        self.com_pipe.subscribe_observer(MessageObserver(func=self._observer_esc_tick, arb_id=SystemTick.id()))

        # Register the cleanup method
        atexit.register(self._destroy)

    @property
    def com_pipe(self) -> CANPipe:
        return self._data_pipe

    @property
    def is_online(self) -> bool:
        return self._online

    @online_checker
    def ping(self) -> bool:
        """
        Communicates with the desired to node to check if it is alive
        Returns:
            True if the node responds, False if not
        """
        # Set up subscription for the expected response
        sub_id = self.com_pipe.subscribe(msg=Ping, qty=1, timeout=3.0)

        # Send the ping to the destination node
        ping = Ping()
        ping.dst.node_id = self._dst_node
        ping.src.node_id = NodeID.NODE_PC

        logger.debug(f"Sending ping to node {self._dst_node}")
        self.com_pipe.bus.send(ping.as_bus_msg())

        # Wait for the message to arrive or timeout
        rx_msg = self.com_pipe.get_subscription_data(sub_id, terminate=True)
        logger.debug(f"Ping {'received' if bool(rx_msg) else 'not received'} from node {self._dst_node}")
        return bool(rx_msg)

    @online_checker
    def switch_mode(self, mode: OrbitESC.Mode) -> bool:
        """
        Attempts to switch the ESC into the desired mode
        Args:
            mode: Which mode to go into

        Returns:
            True if the switch was successful, False if not
        """
        if mode not in OrbitESC.Mode.switchable_modes():
            logger.warning(f"Cannot switch directly into {str(mode)} mode")
            return False

        # Set up subscription for the expected response
        sub_id = self.com_pipe.subscribe(msg=SystemMode, timeout=1.0)

        # Set up the mode switch command
        msg = SetSystemMode()
        msg.dst.node_id = self._dst_node
        msg.mode = mode.value

        # Send out the command
        logger.debug(f"Switching to {str(mode)} mode")
        self.com_pipe.bus.send(msg.as_bus_msg())

        # Look through all available messages to find the mode switch occurred
        return self._listen_for_mode_switch(sub_id=sub_id, mode=mode)

    @online_checker
    def get_mode(self) -> Union[OrbitESC.Mode, None]:
        """
        Returns:
            Current mode announced by the ESC
        """
        sub_id = self.com_pipe.subscribe(msg=SystemMode, qty=1, timeout=SystemMode.rate() * 5)
        rx_msg = self.com_pipe.get_subscription_data(sub_id, terminate=True)

        if not rx_msg:
            return None
        else:
            return OrbitESC.Mode(rx_msg[0].mode)

    @online_checker
    def set_speed_reference(self, rpm: Union[float, int]) -> None:
        """
        Sets the speed reference used as the motor control set point
        Args:
            rpm: Desired RPM

        Returns:
            None
        """
        msg = SetMotorSpeed()
        msg.dst.node_id = self._dst_node
        msg.speed = rpm

        logger.debug(f"Setting node {self._dst_node} rpm to: {rpm}")
        self.com_pipe.bus.send(msg.as_bus_msg())

    @online_checker
    def set_config(self, key: int, value: Union[float, int, str]):
        pass

    @online_checker
    def get_config(self, key: int) -> Any:
        pass

    def emergency_stop(self, all_nodes: bool = False) -> bool:
        """
        Communicates an emergency halt message to the current node.
        Args:
            all_nodes: Optionally extend the message to include all nodes

        Returns:
            True if the connected ESC nodes were put into the expected safe state, else False
        """
        # Set up the message to transmit
        msg = EmergencyHalt()
        msg.dst.node_id = NodeID.NODE_ALL.value if all_nodes else self._dst_node

        # Currently the ESC class doesn't track all nodes that are online, so tracking the
        # state of each node to ensure a good transition to a safe state isn't feasible (yet).
        if all_nodes:
            logger.debug("Sending EmergencyHalt to all connected nodes")
            self.com_pipe.bus.send(msg.as_bus_msg())
            return True

        # Listen for the mode status
        sub_id = self.com_pipe.subscribe(msg=SystemMode, timeout=2.0)

        # Send out the halt message
        logger.debug(f"Sending EmergencyHalt to node {self._dst_node.value}")
        self.com_pipe.bus.send(msg.as_bus_msg())

        # Listen for the mode switch
        return self._listen_for_mode_switch(sub_id=sub_id, mode=OrbitESC.Mode.Fault)

    def system_reset(self, all_nodes: bool = False) -> bool:
        """
        Instructs an ESC to do a full system reset
        Args:
            all_nodes: Optionally reset all nodes

        Returns:
            True if the reset occurred, False if not
        """
        # Set up the message to transmit
        msg = SystemReset()
        msg.dst.node_id = NodeID.NODE_ALL.value if all_nodes else self._dst_node

        if all_nodes:
            logger.debug("Sending reset message to all connected nodes")
            self.com_pipe.bus.send(msg.as_bus_msg())

            # See emergency_stop() for reasoning on why this is defaulted to True
            return True

        # Save off the last tick count, then ship the reset command
        logger.debug(f"Sending SystemReset to node {self._dst_node.value}")
        last_reset_tick = copy.copy(self._last_tick)
        self.com_pipe.bus.send(msg.as_bus_msg())
        time.sleep(0.250)

        # Subscribe to the tick message to get the new value when rebooted. This time subscribe after the
        # reset message is sent to prevent aliasing of messages from the last power cycle.
        sub_id = self.com_pipe.subscribe(msg=SystemTick, qty=5, timeout=5.0)

        # Get the reset information
        if packets := self.com_pipe.get_subscription_data(sub_id, terminate=True):
            for pkt in packets:
                if pkt.tick < last_reset_tick:
                    logger.info(f"Node {self._dst_node.value} was reset")
                    return True

        logger.warning(f"Failed to reset node {self._dst_node.value}")
        return False

    def _destroy(self) -> None:
        """
        Teardown function for class cleanup
        Returns:
            None
        """
        logger.debug(f"Tearing down OrbitESC for node {self._dst_node}")
        self._kill_signal.set()
        self._data_pipe.shutdown()

    def _background_thread(self) -> None:
        """
        General purpose thread to handle background tasks that help with processing the ESC data
        Returns:
            None
        """
        while not self._kill_signal.is_set():
            self._notify_signal.wait(timeout=0.1)

            # Do a tick timeout check to ensure the node hasn't dropped off the face of the earth
            if self._online and ((time.time() - self._time_last_online) > (SystemTick.period() * 2)):
                logger.warning(f"Node {self._dst_node} is offline")
                self._online = False

    def _observer_esc_tick(self, can_msg: can.Message) -> None:
        """
        Looks for the SystemTick of the registered node to determine online/offline status
        Args:
            can_msg: Received CAN bus message

        Returns:
            None
        """
        msg = SystemTick().unpack(can_msg)

        if msg.src.node_id == self._dst_node:
            self._time_last_online = time.time()
            self._last_tick = msg.tick
            if not self._online:
                logger.info(f"Node {self._dst_node} is online")
                self._online = True

    def _listen_for_mode_switch(self, sub_id: uuid.UUID, mode: OrbitESC.Mode) -> bool:
        """
        Listens for the SystemMode packet to determine if a mode change has occurred
        Args:
            sub_id: Subscription ID associated with the mode switching event. Assumes a timeout exists.
            mode: The expected new mode

        Returns:
            True if the mode was achieved, False if not
        """
        while rx_msgs := self.com_pipe.get_subscription_data(sub_id, block=True):
            for msg in rx_msgs:
                if msg.mode == mode.value:
                    logger.debug(f"Mode switch success")
                    return True

        logger.warning(f"Failed switch to {str(mode)} mode")
        return False
