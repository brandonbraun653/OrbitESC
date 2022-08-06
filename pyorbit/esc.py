# **********************************************************************************************************************
#   FileName:
#       esc.py
#
#   Description:
#       Interface to control an ESC
#
#   07/24/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import atexit
import can
import time
from loguru import logger
from pyorbit.messages import NodeID, SystemTick
from threading import Event, Thread
from typing import Any, Union
from pyorbit.pipe import CANPipe, MessageObserver


class OrbitESC:

    def __init__(self, dst_node: NodeID, this_node: NodeID = NodeID.NODE_PC, device: str = "can0", bit_rate: int = 1000000):
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

        # Start up the background thread
        self._kill_signal = Event()
        self._notify_signal = Event()
        self._bkgd_thread = Thread(target=self.__background_thread)
        self._bkgd_thread.start()

        # Register known observers
        self.com_pipe.subscribe(MessageObserver(func=self._observer_esc_tick, arb_id=SystemTick.id(), persistent=True))

        # Register the cleanup method
        atexit.register(self._destroy)

    @property
    def com_pipe(self) -> CANPipe:
        return self._data_pipe

    def arm(self) -> None:
        pass

    def disarm(self) -> None:
        pass

    def engage(self) -> None:
        pass

    def disengage(self) -> None:
        pass

    def emergency_stop(self, all_nodes: bool = False) -> None:
        pass

    def set_speed_reference(self, rpm: Union[float, int]) -> None:
        pass

    def get_config(self, key: int) -> Any:
        pass

    def set_config(self, key: int, value: Union[float, int, str]):
        pass

    def _destroy(self) -> None:
        """
        Teardown function for class cleanup
        Returns:
            None
        """
        logger.debug(f"Tearing down OrbitESC for node {self._dst_node}")
        self._data_pipe.shutdown()

    def __background_thread(self) -> None:
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

    def _observer_esc_tick(self, can_msg: can.Message, timeout: bool) -> None:
        """ Looks for the SystemTick of the registered node to determine online/offline status """
        msg = SystemTick().unpack(can_msg)

        if msg.src.node_id == self._dst_node:
            self._time_last_online = time.time()
            if not self._online:
                logger.info(f"Node {self._dst_node} is online")
                self._online = True

