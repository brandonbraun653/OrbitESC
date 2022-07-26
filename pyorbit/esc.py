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


class OrbitESC:

    def __init__(self, node_id: NodeID):
        self._node_id = node_id

