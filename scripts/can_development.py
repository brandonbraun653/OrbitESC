
import time
from pyorbit.pipe import CANPipe
from pyorbit.messages import NodeID, MotorSpeed, PowerSupplyVoltage
from pyorbit.plotter import LivePlotter
from pyorbit.esc import OrbitESC
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # plotter = LivePlotter(message=MotorSpeed(), attr_key="speed", time_key="tick")
    plotter = LivePlotter(message=PowerSupplyVoltage(), attr_key="vdd")
    # plt.show(block=False)
    # plotter.live_animate()

    esc = OrbitESC(dst_node=NodeID.NODE_0)
    esc.com_pipe.subscribe(plotter.observer_handle)

    time.sleep(5)
