import time
import sys
from pyorbit.pipe import CANPipe
from pyorbit.messages import NodeID, MotorSpeed, PowerSupplyVoltage
from pyorbit.plotter import LivePlotter
from pyorbit.esc import OrbitESC
import matplotlib.pyplot as plt
from loguru import logger

if __name__ == "__main__":
    logger.remove()
    logger.add(sys.stderr, level="TRACE")

    plotter = LivePlotter(message=MotorSpeed(), attr_key="speed", time_key="tick")
    # plotter = LivePlotter(message=PowerSupplyVoltage(), attr_key="vdd")
    # plt.show(block=False)

    esc = OrbitESC(dst_node=NodeID.NODE_1)
    esc.com_pipe.subscribe_observer(plotter.observer_handle)
    time.sleep(1)

    esc.system_reset()

    time.sleep(1)
    esc.switch_mode(OrbitESC.Mode.Idle)
    time.sleep(1)
    esc.switch_mode(OrbitESC.Mode.Armed)
    time.sleep(1)
    esc.switch_mode(OrbitESC.Mode.Run)
    time.sleep(10)
    esc.emergency_stop()

    # plotter.live_animate()
    # time.sleep(5)
