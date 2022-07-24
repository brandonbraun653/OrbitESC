
from pyorbit.esc import OrbitESC
from pyorbit.messages import NodeID
from loguru import logger

if __name__ == "__main__":
    esc = OrbitESC(can_device="can0", bit_rate=1000000)
    esc.connect(node_id=NodeID.NODE_0)
    esc.shutdown()
