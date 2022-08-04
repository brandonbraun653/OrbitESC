# **********************************************************************************************************************
#   FileName:
#       esc.py
#
#   Description:
#       Interface to control an ESC
#
#   07/24/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

from pyorbit.messages import NodeID
from typing import Any, Union


class OrbitESC:

    def __init__(self, node_id: NodeID, ):
        self._node_id = node_id

    def connect(self) -> bool:
        pass

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
