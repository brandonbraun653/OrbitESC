
import time
from pyorbit.pipe import CANPipe
from pyorbit.messages import NodeID
from pyorbit.plotter import LivePlotter
from loguru import logger
import matplotlib.pyplot as plt

if __name__ == "__main__":
    plotter = LivePlotter(arb_id=0x20)

    esc = CANPipe(can_device="can0", bit_rate=1000000)
    esc.ping(node_id=NodeID.NODE_0)
    esc.subscribe(plotter.observer_handle)

    plt.show(block=False)
    plotter.redraw_from_main()
    time.sleep(3)
    esc.shutdown()


# Cleaner data plots
# Accumulate and dump data
# Properly calculate phase currents and power supply voltage
