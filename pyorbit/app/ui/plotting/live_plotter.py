import collections
import random
import time
import math
from typing import List

import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal
from pyqtgraph import PlotWidget
from pyqtgraph.Qt import QtCore

from system_data_pb2 import SystemDataId


class LivePlotManager(QtCore.QObject):
    """ Manager class for handling live plotting requests from other widgets """

    def __init__(self, parent: QtCore.QObject):
        super().__init__(parent)
        self.plotters = []

    def request_plot(self) -> None:
        plotter = DynamicPlotter(sample_interval=0.01, time_window=10.)
        plotter.sigClosed.connect(lambda: self.remove_plotter(plotter))
        self.plotters.append(plotter)

    def remove_plotter(self, plotter) -> None:
        print("Removing plotter")
        self.plotters.remove(plotter)


class DataThread(QThread):
    sigUpdate = pyqtSignal()

    def __init__(self, data_items: List[SystemDataId], sample_interval=0.1, time_window=10.):
        QThread.__init__(self)

        # Data stuff
        self._period_ms = int(sample_interval * 1000)
        self._buf_size = int(time_window / sample_interval)
        self.data_buffer = collections.deque([0.0] * self._buf_size, self._buf_size)
        self.x = np.linspace(-time_window, 0.0, self._buf_size)
        self.y = np.zeros(self._buf_size, dtype=float)

        # TODO: Maybe don't use a timer. Register a listener with the serial class.
        # Use the listener to actually drive the update plot event.

        # Start a timer at the requested interval to perform the plot updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(self._period_ms)

    def update_plot(self):
        # Grab the latest update for each field
        self.data_buffer.append(self.getdata())
        self.y[:] = self.data_buffer
        self.sigUpdate.emit()


class DynamicPlotter(PlotWidget):
    sigClosed = pyqtSignal()

    def __init__(self, data_items: List[SystemDataId], sample_interval=0.1, time_window=10., size=(800, 350)):
        PlotWidget.__init__(self)

        # TODO: Need to check if the device is actually connected before really creating the plot

        # PyQtGraph stuff
        self.setWindowTitle("dynamic plotting")
        self.show()
        self.resize(*size)
        self.showGrid(x=True, y=True)
        self.setLabel('left', 'amplitude', 'V')
        self.setLabel('bottom', 'time', 's')

        # TODO: Create multiple curves
        self.curve = self.plot(pen=(255, 0, 0))

        # DataThread
        self.data_thread = DataThread(sample_interval, time_window)
        self.data_thread.sigUpdate.connect(self.update_plot)
        self.data_thread.start()

    def closeEvent(self, a0):
        super().closeEvent(a0)
        self.data_thread.quit()
        self.sigClosed.emit()

    def update_plot(self):
        # Update each curve from the data thread
        self.curve.setData(self.data_thread.x, self.data_thread.y)
