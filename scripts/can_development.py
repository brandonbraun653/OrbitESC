
import time
from pyorbit.pipe import CANPipe
from pyorbit.messages import NodeID, MotorSpeed
from pyorbit.plotter import LivePlotter
import matplotlib.pyplot as plt

if __name__ == "__main__":
    plotter = LivePlotter(message=MotorSpeed(), attr_key="speed", time_key="tick")
    plt.show(block=False)

    esc = CANPipe(can_device="can0", bit_rate=1000000)
    esc.ping(node_id=NodeID.NODE_0)
    esc.subscribe(plotter.observer_handle)

    plotter.live_animate()

    time.sleep(3)
    esc.shutdown()
