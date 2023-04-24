# **********************************************************************************************************************
#   FileName:
#       live_data_plotter.py
#
#   Description:
#       Utility for live plotting various pieces of data from the OrbitESC module
#
#   04/16/2023 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************
from collections import deque
import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
from loguru import logger
from pyorbit.observer import MessageObserver
from pyorbit.serial_client import SerialClient
from pyorbit.serial_messages import SystemDataMessage
from pyorbit.serial_pipe import SerialPipe


class MotorCurrentPlotter(MessageObserver):
    """ Utility for live plotting motor current data from the Serial debug port """

    def __init__(self, pipe: SerialPipe):
        """
        Args:
            pipe: SerialPipe with an active connection to the OrbitESC
        """
        super().__init__(self._observer_func, SystemDataMessage)
        self._com_pipe = pipe
        self._com_pipe.subscribe_observer(self)

        length = 1000
        self.y_axis = deque(np.zeros(length), maxlen=length)
        self.x_axis = deque(np.zeros(length), maxlen=length)

    def _observer_func(self, msg: SystemDataMessage) -> None:
        """
        Callback function to handle incoming messages from the OrbitESC

        Args:
            msg: Message object from the OrbitESC
        """
        adc_data = msg.convert_to_message_type()
        self.x_axis.append(adc_data.timestamp / 1e6)
        self.y_axis.append(adc_data.phase_a)

    def _graphics_thread(self):
        """ Thread to handle the live plotting of data """
        pass


if __name__ == "__main__":
    import time

    client = SerialClient(port="/dev/ttyUSB0", baudrate=2000000)
    plotter = MotorCurrentPlotter(client.com_pipe)

    pg.setConfigOptions(antialias=True)
    win = pg.GraphicsLayoutWidget(show=True, title="Motor Current Plotter")
    p = win.addPlot()
    p.setRange(yRange=[-1, 1])
    curve = p.plot()

    def update():
        global curve, plotter
        y = np.array(plotter.y_axis)
        x = np.array(plotter.x_axis)
        new_data = np.vstack((x, y)).transpose()
        curve.setData(new_data)
        p.enableAutoRange('x', True)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    QtWidgets.QApplication.instance().exec_()

    client.close()
