
import time
from pyorbit.pipe import CANPipe
from pyorbit.messages import NodeID, MotorSpeed, PowerSupplyVoltage
from pyorbit.plotter import LivePlotter
from pyorbit.esc import OrbitESC
import matplotlib.pyplot as plt


if __name__ == "__main__":
    plotter = LivePlotter(message=MotorSpeed(), attr_key="speed", time_key="tick")
    # plotter = LivePlotter(message=PowerSupplyVoltage(), attr_key="vdd")
    plt.show(block=False)

    esc = OrbitESC(dst_node=NodeID.NODE_0)
    esc.com_pipe.subscribe_observer(plotter.observer_handle)

    time.sleep(1)
    esc.ping()

    plotter.live_animate()
    time.sleep(5)
