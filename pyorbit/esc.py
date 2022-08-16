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
from threading import Event, Thread
from typing import Any, Union
from pyorbit.pipe import CANPipe, MessageObserver
from pyorbit.messages import NodeID, Ping, SystemTick


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
        self._bkgd_thread = Thread(target=self._background_thread, daemon=True,
                                   name=f"OrbitESC_BackgroundThread_Node{dst_node}")
        self._bkgd_thread.start()

        # Register known observers
        self.com_pipe.subscribe_observer(MessageObserver(func=self._observer_esc_tick, arb_id=SystemTick.id()))

        # Register the cleanup method
        atexit.register(self._destroy)

    @property
    def com_pipe(self) -> CANPipe:
        return self._data_pipe

    def ping(self) -> bool:
        """
        Communicates with the desired to node to see if it's alive
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
            if not self._online:
                logger.info(f"Node {self._dst_node} is online")
                self._online = True
